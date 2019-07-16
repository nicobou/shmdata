# PLUGIN

set(PLUGIN_NAME "plugin-watcher")
set(PLUGIN_DESCRIPTION "Watcher Plugin")

option(PLUGIN_WATCHER "${PLUGIN_DESCRIPTION}" TRUE)
add_feature_info("${PLUGIN_NAME}" PLUGIN_WATCHER "${PLUGIN_DESCRIPTION}")

if (PLUGIN_WATCHER)

    add_library(watcher SHARED
        watcher.cpp
        )

    add_dependencies(watcher ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_watcher check_watcher.cpp)
    add_test(check_watcher check_watcher)

    # INSTALL

    install(TARGETS watcher LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
