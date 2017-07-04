# PLUGIN

set(PLUGIN_NAME "plugin-av")
set(PLUGIN_DESCRIPTION "AV recorder/player Plugin")

option(PLUGIN_AV "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_AV "${PLUGIN_DESCRIPTION}")

if (PLUGIN_AV)

    add_library(avrec SHARED
            avrec.cpp
    )

    add_dependencies(avrec ${SWITCHER_LIBRARY})

    add_library(avplayer SHARED
            avplayer.cpp
    )

    add_dependencies(avrec ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_av check_avrecplay.cpp)
    add_test(check_av check_av)

    # INSTALL

    install(TARGETS avrec avplayer LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()