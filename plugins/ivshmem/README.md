# IVShMem plugin

IVShMem stands for Inter-VM shared memory. This module allows OpenRAVE to transmit data structures to special-purpose virtual machines via the [POSIX shared memory](https://man7.org/linux/man-pages/man7/shm_overview.7.html), which is kind of like inter-process communication.

This module is a collision checking module. The receiving party should implement an FCL-compatible collision checking routines.

## Class Structure

The entrypoint of the plugin is `ivshmem.cpp`. It constructs the `IVShMemInterface` class which implements the `CollisionCheckerBase` interface, which creates a communication channel with the shared memory region.
This communication channel is in the form of the `IVShMemServer` class, which provides a `read(2)`/`write(2)`-like interface, as well as a socket to send and receive interrupts.

By default, the shared memory is created at `/dev/shm/ivshmem` and the interrupt socket is created at `tmp/ivshmem_socket`.
