# PLUGIN

pkg_check_modules(LIBLO liblo)

set(PLUGIN_NAME "plugin-osc")
set(PLUGIN_DESCRIPTION "OSC Plugin")

option(PLUGIN_OSC "${PLUGIN_DESCRIPTION}" ${LIBLO_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_OSC "${PLUGIN_DESCRIPTION}")

if (PLUGIN_OSC)

    pkg_check_modules(LIBLO REQUIRED liblo)

    add_compile_options(${LIBLO_CFLAGS})

    include_directories(
        ${LIBLO_INCLUDE_DIRS}
    )

    link_libraries(
        ${LIBLO_LIBRARIES}
    )

    # OSC-CTRL

    add_library(osc_ctrl_server SHARED
        osc-ctrl-server.cpp
        )

    add_dependencies(osc_ctrl_server ${SWITCHER_LIBRARY})

    # OSC-to-SHM

    add_library(osc_to_shmdata SHARED
        osc-to-shmdata.cpp
        )

    add_dependencies(osc_to_shmdata ${SWITCHER_LIBRARY})

    # SHM-to-OSC

    add_library(shmdata_to_osc SHARED
        shmdata-to-osc.cpp
        )

    add_dependencies(shmdata_to_osc ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_osc check_osc.cpp)
    add_test(check_osc check_osc)

    # INSTALL

    install(TARGETS osc_ctrl_server osc_to_shmdata shmdata_to_osc LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()