# PLUGIN

find_package(VRPN)

set(PLUGIN_NAME "plugin-vrpn")
set(PLUGIN_DESCRIPTION "VRPN Plugin")

option(PLUGIN_VRPN "${PLUGIN_DESCRIPTION}" ${VRPN_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_VRPN "${PLUGIN_DESCRIPTION}")

if (PLUGIN_VRPN)

    # VRPN
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vrpn-lib/cmake")

    # VRPN Mandatory dependency
    find_package(VRPN REQUIRED)

    # PLUGINS

    # Source
    add_library(vrpnsrc SHARED
        vrpn-source-plugin.cpp
        source/vrpn-source.cpp
        source/devices/device.cpp
        source/devices/analog-device.cpp
        source/devices/button-device.cpp
        source/devices/tracker-device.cpp
        source/devices/properties/property-base.cpp
        source/devices/properties/bool-property.cpp
        source/devices/properties/double-property.cpp
        )

    add_dependencies(vrpnsource ${SWITCHER_LIBRARY})

    target_include_directories(vrpnsrc PRIVATE
        ${VRPN_INCLUDE_DIRS}
        )

    target_link_libraries(vrpnsrc
        ${VRPN_LIBRARIES}
        )

    # Sink
    add_library(vrpnsink SHARED
        vrpn-sink-plugin.cpp
        sink/vrpn-sink.cpp
        )

    add_dependencies(vrpnsink ${SWITCHER_LIBRARY})

    target_include_directories(vrpnsink PRIVATE
        ${COMMON_INCLUDE_DIRS}
        ${VRPN_INCLUDE_DIRS}
        )

    target_link_libraries(vrpnsink
        ${SWITCHER_LIBRARY}
        ${VRPN_SERVER_LIBRARIES}
        )

    # INSTALL

    install(TARGETS vrpnsrc vrpnsink LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()