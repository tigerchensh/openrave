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

#ifndef OPENRAVE_FD_HPP
#define OPENRAVE_FD_HPP

#include <unistd.h>

// RAII file descriptors that closes itself upon scope exit.
// This is useful to avoid file descriptor leaks.
// Currently, only file descriptors that close with `close(2)` is supported.
struct FileDescriptor final {
public:
    // Default constructor creates a FileDescriptor with an invalid value (-1)
    FileDescriptor() noexcept : _fd(-1) {}

    // Transfers ownership of the dile descriptor to this object.
    explicit FileDescriptor(int fd) noexcept : _fd(fd) {}

    // Constructs the file descriptor with the given creation function and arguments.
    template <typename Ctor, typename... Args>
    FileDescriptor(Ctor&& ctor, Args&&... args) noexcept : _fd(ctor(std::forward<Args>(args)...)) {}

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor(FileDescriptor&& other) noexcept : _fd(other._fd) {
        other._fd = -1;
    }

    ~FileDescriptor() noexcept {
        if (_fd > -1) {
            ::close(_fd);
        }
    }

    int get() const noexcept {
        return _fd;
    }

    operator bool() const noexcept {
        return (_fd > -1);
    }

    FileDescriptor& operator=(FileDescriptor&& other) noexcept = default;

private:
    int _fd;
};

#endif // OPENRAVE_FD_HPP