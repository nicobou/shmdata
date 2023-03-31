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
target_compile_options(${SHMDATA_LIBRARY} PUBLIC -DSHMDATA_VERSION_STRING="${SHMDATA_VERSION_STRING}" )

# Pkg-config
if (UNIX)
    include(FindPkgConfig QUIET)
    if (PKG_CONFIG_FOUND)
        configure_file("shmdata.pc.in" "shmdata-${SHMDATA_API_VERSION}.pc" @ONLY)
        # Install Pkg-config file for the 'dev' package
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/shmdata-${SHMDATA_API_VERSION}.pc"
          COMPONENT dev
          DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
        # Install Pkg-config file for the 'lib' package
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/shmdata-${SHMDATA_API_VERSION}.pc"
          COMPONENT lib
          DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
    endif ()
endif ()

install(TARGETS ${SHMDATA_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT lib)

install(TARGETS ${SHMDATA_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT lib)
  
install(DIRECTORY ${HEADER_DIR_INCLUDES}
    DESTINATION include/shmdata-${SHMDATA_API_VERSION}/shmdata
    COMPONENT dev
    FILES_MATCHING # install only matched files
    PATTERN "*.hpp"
    PATTERN "*.h" 
    )
