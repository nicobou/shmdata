# PLUGIN

set(PLUGIN_NAME "plugin-vrpn")
set(PLUGIN_DESCRIPTION "VRPN Plugin")

option(PLUGIN_VRPN "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_VRPN "${PLUGIN_DESCRIPTION}")

if (PLUGIN_VRPN)

    # VRPN

    include(ExternalProject)
    ExternalProject_Add(vrpn
        URL ${CMAKE_CURRENT_SOURCE_DIR}/external/vrpn
        UPDATE_COMMAND "" # Prevents rebuilds on every update
        INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
        CMAKE_ARGS -DVRPN_BUILD_SERVERS=OFF -DVRPN_BUILD_CLIENTS=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/vrpn-install
        )

    set(VRPN_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/vrpn-install/include)
    set(VRPN_LIBRARIES_DIR ${CMAKE_CURRENT_BINARY_DIR}/vrpn-install/lib)

    include_directories(
        ${VRPN_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    link_directories(
        ${VRPN_LIBRARIES_DIR}
    )

    link_libraries(
        ${VRPN_LIBRARIES_DIR}/libquat.a
    )

    # WARNINGS
    add_definitions(-Wno-error=unused-variable)

    # PLUGINS

    set(SHARED_SRC
        shared/device.cpp
    )

    # Source
    add_library(vrpnsrc SHARED
        ${SHARED_SRC}
        vrpn-source-plugin.cpp
        source/vrpn-source.cpp
        source/devices/source-device.cpp
        source/devices/analog-source-device.cpp
        source/devices/button-source-device.cpp
        source/devices/tracker-source-device.cpp
        source/devices/properties/property-base.cpp
    )

    add_dependencies(vrpnsrc ${SWITCHER_LIBRARY})
    add_dependencies(vrpnsrc vrpn)

    target_link_libraries(vrpnsrc
        PRIVATE ${VRPN_LIBRARIES_DIR}/libvrpn.a)

    # Sink
    add_library(vrpnsink SHARED
        ${SHARED_SRC}
        vrpn-sink-plugin.cpp
        sink/vrpn-sink.cpp
        sink/devices/button/vrpn-button-server.cpp
        sink/devices/sink-device.cpp
        sink/devices/analog-sink-device.cpp
        sink/devices/button-sink-device.cpp
    )

    add_dependencies(vrpnsink ${SWITCHER_LIBRARY})
    add_dependencies(vrpnsink vrpn)

    target_link_libraries(vrpnsink
        PRIVATE ${VRPN_LIBRARIES_DIR}/libvrpnserver.a)

    # UTILS

    add_executable(debug-client utils/debug-client.cpp)
    add_dependencies(debug-client vrpn)
    target_link_libraries(debug-client PRIVATE ${VRPN_LIBRARIES_DIR}/libvrpn.a)

    add_executable(debug-server utils/debug-server.cpp)
    add_dependencies(debug-server vrpn)
    target_link_libraries(debug-server PRIVATE ${VRPN_LIBRARIES_DIR}/libvrpnserver.a)

    # TEST

    add_executable(check_vrpn_plugin tests/check_vrpn_plugin.cpp)
    add_dependencies(check_vrpn_plugin vrpn)
    add_test(check_vrpn_plugin check_vrpn_plugin)

    # INSTALL

    install(TARGETS vrpnsrc vrpnsink LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
