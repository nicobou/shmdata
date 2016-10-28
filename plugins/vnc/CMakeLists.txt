# PLUGIN

pkg_check_modules(VNC libvncclient)

set(PLUGIN_NAME "plugin-vnc")
set(PLUGIN_DESCRIPTION "VNC Plugin")

option(PLUGIN_VNC "${PLUGIN_DESCRIPTION}" ${VNC_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_VNC "${PLUGIN_DESCRIPTION}")

if (PLUGIN_VNC)

    pkg_check_modules(VNC REQUIRED libvncclient)

    add_compile_options(${VNC_CFLAGS})

    include_directories(
        ${VNC_INCLUDE_DIRS}
    )

    link_libraries(
        ${VNC_LIBRARIES}
    )

    add_library(vncplugin SHARED
        vnc_client.cpp
        )

    add_dependencies(vncplugin ${SWITCHER_LIBRARY})

    # TEST

    # INSTALL

    install(TARGETS vncplugin LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()