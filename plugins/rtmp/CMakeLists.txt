# PLUGIN

set(PLUGIN_NAME "plugin-rtmp")
set(PLUGIN_DESCRIPTION "RTMP Plugin")

option(PLUGIN_RTMP "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_RTMP "${PLUGIN_DESCRIPTION}")

if (PLUGIN_RTMP)

    add_library(rtmp SHARED
        rtmp.cpp
        )

    add_dependencies(rtmp ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_rtmp check_rtmp.cpp)
    add_test(check_rtmp check_rtmp)

    # INSTALL

    install(TARGETS rtmp LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
