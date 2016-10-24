# PLUGIN

set(PLUGIN_NAME "plugin-systemusage")
set(PLUGIN_DESCRIPTION "System Usage Plugin")

if(UNIX AND NOT OSX)
    set(ENABLED TRUE)
endif()

option(PLUGIN_SYSTEMUSAGE "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_SYSTEMUSAGE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_SYSTEMUSAGE)

    add_library(systemusage SHARED
        systemusage.cpp
        )

    add_dependencies(systemusage ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_systemusage_plugin check_systemusage_plugin.cpp)
    add_test(check_systemusage_plugin check_systemusage_plugin)

    # INSTALL

    install(TARGETS systemusage LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()