# PLUGIN

set(PLUGIN_NAME "plugin-crashtest")
set(PLUGIN_DESCRIPTION "Crash test Plugin")

if(UNIX AND NOT OSX)
    set(ENABLED TRUE)
endif()

option(PLUGIN_CRASHTEST "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_CRASHTEST "${PLUGIN_DESCRIPTION}")

if (PLUGIN_CRASHTEST)

  add_library(crashtest SHARED
    crashtest.cpp
    )

    add_dependencies(crashtest ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_crashtest_plugin check_crashtest_plugin.cpp)
    add_test(check_crashtest_plugin check_crashtest_plugin)

    # INSTALL

    install(TARGETS crashtest LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
