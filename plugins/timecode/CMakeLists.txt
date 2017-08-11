# PLUGIN

pkg_check_modules(JACK REQUIRED jack)
pkg_check_modules(LTC REQUIRED ltc)

if(JACK_FOUND AND LTC_FOUND)
    set(ENABLED TRUE)
endif()


set(PLUGIN_NAME "plugin-timecode")
set(PLUGIN_DESCRIPTION "Timecode handler for synchronization on audio stream")

option(PLUGIN_TIMECODE "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_TIMECODE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_TIMECODE)

    add_compile_options(${JACK_CFLAGS} ${LTC_CFLAGS})

    link_libraries(
        ${JACK_LIBRARIES}
        ${LTC_LIBRARIES}
    )

    # LTC-to-JackTransport
    add_library(ltctojack SHARED
                ltc-to-jack.cpp
    )

    add_dependencies(ltctojack ${SWITCHER_LIBRARY})

    # Test
    add_executable(check_ltc_to_jack check_ltc_to_jack.cpp)
    add_test(check_ltc_to_jack check_ltc_to_jack)

    # LTC-to-JackTransport
    add_library(ltcsource SHARED
                ltc-source.cpp
    )

    add_dependencies(ltcsource ${SWITCHER_LIBRARY})

    # Test
    add_executable(check_ltc_source check_ltc_source.cpp)
    add_test(check_ltc_source check_ltc_source)


    # Install
    install(TARGETS ltctojack ltcsource LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
