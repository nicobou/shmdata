# PLUGIN

set(PLUGIN_NAME "plugin-ladspa")
set(PLUGIN_DESCRIPTION "LADSPA Plugin")

option(PLUGIN_LADSPA "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_LADSPA "${PLUGIN_DESCRIPTION}")

if (PLUGIN_LADSPA)

    add_library(ladspa SHARED
        ladspa.cpp
        check_ladspa.cpp)

    add_dependencies(ladspa ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_ladspa check_ladspa.cpp)
    configure_file(check_ladspa.json check_ladspa.json COPYONLY)
    add_test(check_ladspa check_ladspa)

    # INSTALL

    install(TARGETS ladspa LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
