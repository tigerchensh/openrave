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
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <functional>
#include <stdexcept>

#include "ivshmem_server.hpp"

#include <openrave/logging.h>

const std::string IVShMemServer::_shmem_path = "ivshmem";
const std::string IVShMemServer::_sock_path = "/tmp/ivshmem_socket";

IVShMemServer::IVShMemServer()
    : _mmap(nullptr)
    , _stop(false)
    , _shmem_size(1024 * 1024)
    , _shmem_fd(::shm_open(_shmem_path.c_str(), O_RDWR | O_CREAT, S_IRWXU)) {

    // Prepare the shared memory region.
    ::ftruncate(_shmem_fd, _shmem_size);
    _mmap = ::mmap(NULL, _shmem_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shmem_fd, 0);
    if (_mmap == MAP_FAILED) {
        throw std::runtime_error("Failed to map memory of ivshmem.");
    }

    _thread = std::thread(std::bind(&IVShMemServer::_Thread, this));
}

IVShMemServer::~IVShMemServer() {
    _stop = true;
    _thread.join();
    ::munmap(_mmap, _shmem_size);
    ::shm_unlink(_shmem_path.c_str());
    ::close(_shmem_fd);
}

void IVShMemServer::_Thread() try {
    // Create listening socket for interrupts
    struct sockaddr_un local;
    int sock_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd == -1) {
        throw std::runtime_error("Failed to create socket at " + _sock_path);
    }
    ::strncpy(local.sun_path, _sock_path.c_str(), sizeof(local.sun_path));
    ::unlink(local.sun_path);
    int len = ::strlen(local.sun_path) + sizeof(local.sun_family);
    if (::bind(sock_fd, (struct sockaddr*)&local, len) == -1) {
        throw std::runtime_error("Failed to bind socket to address.");
    }

    // Set up epoll
    static constexpr size_t MAX_EPOLL_EVENTS = 2;
    int ep_fd = ::epoll_create(MAX_EPOLL_EVENTS);
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP;
    ev.data.fd = sock_fd;
    if (::epoll_ctl(ep_fd, EPOLL_CTL_ADD, sock_fd, &ev) == -1) {
        throw std::runtime_error("Failed to register epoll event.");
    }
    ::epoll_event events[MAX_EPOLL_EVENTS];

    int64_t guest_id = 0;

    // Some buffers as scratch space
    static constexpr size_t BUFFER_SIZE = 1024 * 8;
    std::array<char, BUFFER_SIZE> buffer;
    while (!_stop) {
        int num_fds = ::epoll_wait(ep_fd, events, MAX_EPOLL_EVENTS, 0);
        if (num_fds == 0) {
            throw std::runtime_error("Error caught in epoll_wait.");
        }
        for (int i = 0; i < num_fds; ++i) {
            int fd = events[i].data.fd;
            if (fd == sock_fd) {
                // Event on the socket fd, there's a new guest.
                struct sockaddr_un remote;
                socklen_t t = sizeof(remote);
                int vm_sock = ::accept(sock_fd, (struct sockaddr*)&remote, &t);
                if (vm_sock == -1) {
                    RAVELOG_WARN("Failed to accept a connection on socket.");
                    continue;
                }
                IVShMemPeer peer;
                peer.sock_fd = vm_sock;
                peer.id = guest_id++;
                // Send the ID of the peer to the peer.
                ssize_t bytes = ::send(peer.sock_fd, &peer.id, sizeof(peer.id), 0);
                if (bytes != sizeof(peer.id)) {
                    RAVELOG_WARN("Failed to send ID to peer.");
                }

                // Send control commands to peer.
                struct cmsghdr* cmsg;
                char control[CMSG_SPACE(sizeof(int))];
                struct iovec iov[1];
                struct msghdr msg = {0, 0, iov, 1, control, sizeof(control), 0};

            }
        }
    }
} catch (const std::exception& e) {
    RAVELOG_ERROR("%s", e.what());
}
