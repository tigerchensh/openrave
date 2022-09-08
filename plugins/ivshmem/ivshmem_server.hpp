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
#include <thread>
#include <vector>

class IVShMemServer final {
public:
    IVShMemServer();
    ~IVShMemServer() {}

private:
    void _Thread();
    
    // Main loop thread
    std::thread _thread;
    bool _stop; // Signals the thread to stop.

    // Shared memory
    static const std::string _shmem_path;
    int _shmem_fd;
    size_t _shmem_size;
    void* _mmap;

    // Socket for interrupts, only when emulating.
    static const std::string _sock_path;

    static constexpr int IVSHMEM_VECTOR_COUNT = 2;
    struct IVShMemPeer final {
        int sock_fd;
        int64_t id;
        int vectors[IVSHMEM_VECTOR_COUNT];
    };
    std::vector<IVShMemPeer> _peers;
};

#endif // OPENRAVE_IVSHMEM_SERVER_HPP
