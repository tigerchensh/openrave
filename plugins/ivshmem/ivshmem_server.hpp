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

#ifndef OPENRAVE_IVSHMEM_SERVER_HPP
#define OPENRAVE_IVSHMEM_SERVER_HPP

#include <atomic>
#include <string>
#include <vector>

#include "fd.hpp"

class IVShMemServer final {
public:
    IVShMemServer();
    ~IVShMemServer();

    /// \brief Main thread loop function. The loop should be run in the class containing this class.
    void Thread();

    void Stop() noexcept { _stop = true; }

    /// This object is in an invalid state if:
    /// - Shared memory file descriptor is invalid(-1), or
    /// - Interrupt socket is invalid(-1), or
    /// - It has been stopped.
    operator bool() const noexcept {
        return !(_stop || (_shmem_fd < 0) || (_sock_fd < 0));
    }

private:
    // Initializes the socket that listens for interrupts.
    // Throws exceptions if the socket can't be bound to a path.
    void _InitSocket();

    // Initializes the shared memory
    void _InitSharedMemory();

    // Add a new guest to the peers list
    void _NewGuest(int64_t guest_id);

    void _RemoveGuest(int64_t guest_id);

    /// \brief Sends a signal to the peer. `message` must be a positive value if it is a control signal; otherwise it is ignored.
    /// Usually this message is the fd of the shared memory.
    /// If peer_id is -1, then message is broadcast.
    /// Returns the result of ::sendmsg.
    static int _ShMem_SendMsg(int sock_fd, int64_t peer_id, int message) noexcept;

    static int _ShMem_RecvMsg(int sock_fd, int64_t& peer_id, int& message) noexcept;

private:
    std::atomic_bool _stop; // Signals the thread to stop.

    // Shared memory
    static const std::string _shmem_path;
    FileDescriptor _shmem_fd;
    size_t _shmem_size;
    void* _mmap;

    // Socket for interrupts, only when emulating.
    static const std::string _sock_path;
    FileDescriptor _sock_fd;

    static constexpr int IVSHMEM_VECTOR_COUNT = 2;
    struct IVShMemPeer final {
        FileDescriptor sock_fd;
        int64_t id;
        int vectors[IVSHMEM_VECTOR_COUNT]; // File descriptors
    };
    std::vector<IVShMemPeer> _peers;
};

#endif // OPENRAVE_IVSHMEM_SERVER_HPP
