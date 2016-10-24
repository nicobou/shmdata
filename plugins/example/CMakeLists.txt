# PLUGIN

set(PLUGIN_NAME "plugin-example")
set(PLUGIN_DESCRIPTION "Example Plugin")

option(PLUGIN_EXAMPLE "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_EXAMPLE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_EXAMPLE)

    # Dummy

    add_library(dummy SHARED
        dummy-plugin.cpp
        )
    add_dependencies(dummy ${SWITCHER_LIBRARY})

    # Custom Save

    add_library(custom_save SHARED
        custom-save-plugin.cpp
        )

    # TEST

    add_executable(check_example_plugin check_example_plugin.cpp)
    add_test(check_example_plugin check_example_plugin)

    add_executable(check_custom_save check_custom_save_plugin.cpp)
    add_test(check_custom_save check_custom_save)

    # INSTALL

    install(TARGETS dummy custom_save LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()