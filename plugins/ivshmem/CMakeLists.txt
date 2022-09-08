
set(ivshmem_src
    ivshmem.cpp # Entrypoint of the plugin
    ivshmem_server.cpp
    ivshmem_server.hpp
    ivshmem_interface.cpp
    ivshmem_interface.hpp
)

add_library(ivshmem SHARED ${ivshmem_src})
target_link_libraries(ivshmem PRIVATE boost_assertion_failed PUBLIC rt libopenrave)
set_target_properties(ivshmem PROPERTIES
    COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
    LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}"
)
