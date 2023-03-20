# PLUGIN


set(PLUGIN_NAME "plugin-timecode")
set(PLUGIN_DESCRIPTION "Timecode handler for synchronization on audio stream")

option(PLUGIN_TIMECODE "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_TIMECODE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_TIMECODE)

    pkg_check_modules(JACK REQUIRED jack)
    pkg_check_modules(LTC REQUIRED ltc)

    add_compile_options(${JACK_CFLAGS} ${LTC_CFLAGS})

    # jack/types.h requires the following:
    check_cxx_compiler_flag(-Wno-error=deprecated-register CAN_IGNORE_DEPRECATED_REGISTER)
    if(CAN_IGNORE_DEPRECATED_REGISTER)
        add_definitions(-Wno-error=deprecated-register)
    endif()

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


    # LTC-Diff
    add_library(ltcdiff SHARED
                ltc-diff.cpp
    )

    add_dependencies(ltcdiff ${SWITCHER_LIBRARY})

    # Test
    add_executable(check_ltc_diff check_ltc_diff.cpp)
    add_test(check_ltc_diff check_ltc_diff)


    # Install
    install(TARGETS ltctojack ltcsource ltcdiff LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
