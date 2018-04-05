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

    # Method quid

    add_library(meth_quid SHARED
            method-quid.cpp
        )
    add_dependencies(meth_quid ${SWITCHER_LIBRARY})

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

    add_executable(check_property_plugin check_property_plugin.cpp)
    add_test(check_property_plugin check_property_plugin)

    add_executable(check_method_plugin check_method_plugin.cpp)
    add_test(check_method_plugin check_method_plugin)

    add_executable(check_custom_save check_custom_save_plugin.cpp)
    add_test(check_custom_save check_custom_save)

    add_executable(check_signals_plugin check_signals_plugin.cpp)
    add_test(check_signals_plugin check_signals_plugin)

    # INSTALL

    install(TARGETS prop_quid meth_quid custom_save signal_quid LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
