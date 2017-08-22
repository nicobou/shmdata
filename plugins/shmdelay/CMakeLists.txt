# PLUGIN

set(PLUGIN_NAME "plugin-shmdelay")
set(PLUGIN_DESCRIPTION "Buffer delay for shmdata")

option(PLUGIN_SHMDELAY "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_SHMDELAY "${PLUGIN_DESCRIPTION}")

if (PLUGIN_SHMDELAY)

    add_library(shmdelay SHARED
                shm-delay.cpp
    )

    add_dependencies(shmdelay ${SWITCHER_LIBRARY})

    # Test
#    add_executable(check_shmdelay check_shmdelay.cpp)
 #   add_test(check_shmdelay check_shmdelay)

    # Install
    install(TARGETS shmdelay LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
