// -*- coding: utf-8 -*-
// Copyright (C) 2022 Tan Li Boon (liboon.tan@mujin.co.jp)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <errno.h>
#include <endian.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cstdarg>
#include <functional>
#include <stdexcept>

#include <openrave/config.h>
#include <openrave/logging.h>

#include "ivshmem_server.hpp"

#ifndef IVSHMEM_PROTOCOL_VERSION
#define IVSHMEM_PROTOCOL_VERSION 0
#endif // IVSHMEM_PROTOCOL_VERSION

using namespace std::literals;

const std::string IVShMemServer::_shmem_path = "ivshmem";
const std::string IVShMemServer::_sock_path = "/tmp/ivshmem_socket";

IVShMemServer::IVShMemServer()
    : _stop(false)
    , _shmem_fd()
    , _shmem_size(1024 * 1024)
    , _mmap(nullptr)
    , _sock_fd() {
}

IVShMemServer::~IVShMemServer() {
    _stop = true;
    ::munmap(_mmap, _shmem_size);
    ::shm_unlink(_shmem_path.c_str());
}

void IVShMemServer::Thread() try {
    _InitSocket();
    _InitSharedMemory();

    // Set up epoll
    static constexpr size_t MAX_EPOLL_EVENTS = 2;
    auto ep_fd = FileDescriptor(epoll_create, MAX_EPOLL_EVENTS);
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP;
    ev.data.fd = _sock_fd.get();
    if (::epoll_ctl(ep_fd.get(), EPOLL_CTL_ADD, _sock_fd.get(), &ev) == -1) {
        throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
    }
    ::epoll_event events[MAX_EPOLL_EVENTS];

    int64_t guest_id = 0;
    while (!_stop) {
        int num_fds = ::epoll_wait(ep_fd.get(), events, MAX_EPOLL_EVENTS, 0);
        if (num_fds == 0) {
            throw std::runtime_error("Error caught in epoll_wait: "s + strerror(errno));
        }
        for (int i = 0; i < num_fds; ++i) {
            int fd = events[i].data.fd;
            // Event on the socket fd, there's a new guest.
            if (fd == _sock_fd.get()) {
                _NewGuest(guest_id++);
                // Add the guest socket fd to epoll.
                struct epoll_event guestev;
                guestev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP;
                guestev.data.fd = _peers.back().sock_fd.get();
                if (::epoll_ctl(ep_fd.get(), EPOLL_CTL_ADD, guestev.data.fd, &guestev) == -1) {
                    throw std::runtime_error("Failed to register epoll event: "s + strerror(errno));
                }
                continue;
            }
            // Event on guest sockets, message from guests.
            for (const auto& peer : _peers) {
                if (peer.sock_fd == fd) {

                    break;
                }
            }
        }
    }
} catch (const std::exception& e) {
    RAVELOG_ERROR("Exception caught: %s", e.what());
}

void IVShMemServer::_InitSocket() {
    _sock_fd = FileDescriptor(socket, AF_UNIX, SOCK_STREAM, 0);
    if (!_sock_fd) {
        throw std::runtime_error("Failed to create unix socket: "s + strerror(errno));
    }

    struct sockaddr_un local;
    local.sun_family = AF_UNIX;
    ::strncpy(local.sun_path, _sock_path.c_str(), sizeof(local.sun_path));
    int len = ::strlen(local.sun_path) + sizeof(local.sun_family);
    if (::bind(_sock_fd.get(), (struct sockaddr*)&local, len) == -1) {
        throw std::runtime_error("Failed to bind socket to address: "s + strerror(errno));
    }

    if (::listen(_sock_fd.get(), 4) == -1) {
        throw std::runtime_error("Failed to listen on socket: "s + strerror(errno));
    }
}

void IVShMemServer::_InitSharedMemory() {
    _shmem_fd = FileDescriptor(shm_open, _shmem_path.c_str(), O_RDWR | O_CREAT, S_IRWXU);

    // Prepare the shared memory region.
    ::ftruncate(_shmem_fd.get(), _shmem_size);

    _mmap = ::mmap(NULL, _shmem_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shmem_fd.get(), 0);
    if (_mmap == MAP_FAILED) {
        throw std::runtime_error("Failed to map memory of ivshmem: "s + strerror(errno));
    }
}

void IVShMemServer::_NewGuest(int64_t guest_id) {
    IVShMemPeer peer;
    peer.id = guest_id;

    struct sockaddr_un remote;
    socklen_t t = sizeof(remote);
    peer.sock_fd = FileDescriptor(::accept, _sock_fd.get(), reinterpret_cast<struct sockaddr*>(&remote), &t);
    if (!peer.sock_fd) {
        _peers.pop_back();
        throw std::runtime_error("Failed to accept connection on socket: "s + strerror(errno));
    }
    for (int i = 0; i < IVSHMEM_VECTOR_COUNT; ++i) {
        peer.vectors[i] = ::eventfd(0, 0);
        if (peer.vectors[i] == -1) {
            RAVELOG_WARN("Failed to create eventfd: %s", strerror(errno));
        }
    }
    _ShMem_SendMsg(peer.sock_fd.get(), IVSHMEM_PROTOCOL_VERSION, -1);
    _ShMem_SendMsg(peer.sock_fd.get(), peer.id, -1);
    _ShMem_SendMsg(peer.sock_fd.get(), peer.id, _shmem_fd);

    // Advertise new peer to all peers, including itself.
    for (size_t i = 0; i < _peers.size(); ++i) {
        auto& otherpeer = _peers[i];
        for (int j = 0; j < IVSHMEM_VECTOR_COUNT; ++j) {
            _ShMem_SendMsg(otherpeer.sock_fd.get(), peer.id, otherpeer.vectors[j]);
        }
    }

    // Advertise all other peers to the new peer, excluding itself.
    for (size_t i = 0; i < _peers.size() - 1; ++i) {
        auto& otherpeer = _peers[i];
        for (int j = 0; j < IVSHMEM_VECTOR_COUNT; ++j) {
            _ShMem_SendMsg(peer.sock_fd.get(), peer.id, otherpeer.vectors[j]);
        }
    }

    RAVELOG_INFO("Added new peer ID = %d", peer.id);
    _peers.emplace_back(std::move(peer));
}

int IVShMemServer::_ShMem_SendMsg(int sock_fd, int64_t peer_id, int message) noexcept {
    peer_id = htole64(peer_id);
    struct iovec iov = {
        .iov_base = &peer_id,
        .iov_len = sizeof(peer_id),
    };
    struct msghdr msg = {
        .msg_name = NULL,
        .msg_namelen = 0,
        .msg_iov = &iov,
        .msg_iovlen = 1,
        .msg_control = NULL,
        .msg_controllen = 0,
        .msg_flags = 0,
    };

    // If `message` is a value greater than 0, then add it as control content
    if (message >= 0) {
        char control[CMSG_SPACE(sizeof(int))];
        ::memset(control, 0, sizeof(control));
        msg.msg_control = control;
        msg.msg_controllen = sizeof(control);

        struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
        cmsg->cmsg_len = CMSG_LEN(sizeof(int));
        cmsg->cmsg_level = SOL_SOCKET;
        cmsg->cmsg_type = SCM_RIGHTS;
        ::memcpy(CMSG_DATA(cmsg), &message, sizeof(message));
    }

    return ::sendmsg(sock_fd, &msg, 0);
}

int IVShMemServer::_ShMem_RecvMsg(int sock_fd, int64_t& peer_id, int& message) noexcept {
    char control[CMSG_SPACE(sizeof(message))];
    struct iovec iov = {
        .iov_base = &peer_id,
        .iov_len = sizeof(peer_id),
    };
    struct msghdr msg = {
        .msg_name = NULL,
        .msg_namelen = 0,
        .msg_iov = &iov,
        .msg_iovlen = 1,
        .msg_control = control,
        .msg_controllen = sizeof(control),
        .msg_flags = 0,
    };
    ssize_t len = 0;
    do {
        len = ::recvmsg(sock_fd, &msg, 0);
    } while (len == -1 && (errno == EINTR || errno == EAGAIN));
    if (len == -1) {
        return -1; // Some error on socket
    }
    
    // If controllen is less than size of cmsghdr, then message is peer_id.
    if (msg.msg_controllen < sizeof(struct cmsghdr)) {
        message = peer_id;
        return 0;
    }

    // Search messages for content, though we are not really expecting guests to send anything
    for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
        if (cmsg->cmsg_level != SOL_SOCKET || cmsg->cmsg_type != SCM_RIGHTS) {
            if (cmsg->cmsg_len != sizeof(control)) {
                continue;
            }
        }
        ::memcpy(&message, CMSG_DATA(cmsg), sizeof(message));
        return 0;
    }

    return -1; // Nothing with content found, error
}