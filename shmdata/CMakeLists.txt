add_library(${SHMDATA_LIBRARY} SHARED
    cfollower.cpp
    clogger.cpp
    cwriter.cpp
    file-monitor.cpp
    follower.cpp
    reader.cpp
    sysv-sem.cpp
    sysv-shm.cpp
    type.cpp
    unix-socket.cpp
    unix-socket-client.cpp
    unix-socket-protocol.cpp
    unix-socket-server.cpp
    writer.cpp
    )

set(HEADER_INCLUDES
    abstract-logger.hpp
    cfollower.h
    clogger.h
    cwriter.h
    console-logger.hpp
    file-monitor.hpp
    follower.hpp
    reader.hpp
    safe-bool-idiom.hpp
    sysv-sem.hpp
    sysv-shm.hpp
    type.hpp
    unix-socket.hpp
    unix-socket-client.hpp
    unix-socket-protocol.hpp
    unix-socket-server.hpp
    writer.hpp
    )

set_target_properties(${SHMDATA_LIBRARY} PROPERTIES VERSION ${SHMDATA_VERSION_STRING} SOVERSION ${SHMDATA_VERSION_MAJOR})

# Pkg-config
if (UNIX)
    include(FindPkgConfig QUIET)
    if (PKG_CONFIG_FOUND)
        configure_file("shmdata.pc.in" "shmdata-${SHMDATA_API_VERSION}.pc" @ONLY)
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/shmdata-${SHMDATA_API_VERSION}.pc"
            DESTINATION lib/pkgconfig)
    endif ()
endif ()

install(TARGETS ${SHMDATA_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT libraries)

install(FILES ${HEADER_INCLUDES}
    DESTINATION include/shmdata-${SHMDATA_API_VERSION}/shmdata
    COMPONENT headers
    )
