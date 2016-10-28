# PLUGIN

pkg_check_modules(PULSE libpulse-mainloop-glib)

set(PLUGIN_NAME "plugin-pulse")
set(PLUGIN_DESCRIPTION "Pulse Plugin")

option(PLUGIN_PULSE "${PLUGIN_DESCRIPTION}" ${PULSE_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_PULSE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_PULSE)

    pkg_check_modules(PULSE REQUIRED libpulse-mainloop-glib)

    add_compile_options(${PULSE_CFLAGS})

    include_directories(
        ${PULSE_INCLUDE_DIRS}
    )

    link_libraries(
        ${PULSE_LIBRARIES}
    )

    # SRC

    add_library(pulsesrc SHARED
        pulsesrc.cpp
        )

    add_dependencies(pulsesrc ${SWITCHER_LIBRARY})

    # SINK

    add_library(pulsesink SHARED
        pulsesink.cpp
        )

    add_dependencies(pulsesink ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_pulsesrc check_pulsesrc.cpp)
    add_test(check_pulsesrc check_pulsesrc)

    # INSTALL

    install(TARGETS pulsesrc pulsesink LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()