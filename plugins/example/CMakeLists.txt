# PLUGIN

set(PLUGIN_NAME "plugin-example")
set(PLUGIN_DESCRIPTION "Example Plugin")

option(PLUGIN_EXAMPLE "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_EXAMPLE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_EXAMPLE)

    # Property quid

    add_library(prop_quid SHARED
            property-quid.cpp
        )
    add_dependencies(prop_quid ${SWITCHER_LIBRARY})

    # Signal quid

    add_library(signal_quid SHARED
            signal-quid.cpp
            )
    add_dependencies(signal_quid ${SWITCHER_LIBRARY})

    # Custom Save

    add_library(custom_save SHARED
        custom-save-plugin.cpp
        )

    # TEST

    add_executable(check_example_plugin check_example_plugin.cpp)
    add_test(check_example_plugin check_example_plugin)

    add_executable(check_custom_save check_custom_save_plugin.cpp)
    add_test(check_custom_save check_custom_save)

    add_executable(check_signals check_signals.cpp)
    add_test(check_signals check_signals)

    # INSTALL

    install(TARGETS prop_quid custom_save signal_quid LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()