# PLUGIN

pkg_check_modules(JACK jack)

set(PLUGIN_NAME "plugin-jack")
set(PLUGIN_DESCRIPTION "Jack Plugin")

option(PLUGIN_JACK "${PLUGIN_DESCRIPTION}" ${JACK_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_JACK "${PLUGIN_DESCRIPTION}")

if (PLUGIN_JACK)

    pkg_check_modules(JACK REQUIRED jack)

    add_compile_options(${JACK_CFLAGS})

    include_directories(
        ${JACK_INCLUDE_DIRS}
    )

    link_libraries(
	${JACK_LIBRARIES}
    )

    # Shm-to-jack

    add_library(switchershmtojack SHARED
        jack-client.cpp
        shmdata-to-jack.cpp
        )

    add_dependencies(switchershmtojack ${SWITCHER_LIBRARY})

    # Jack-to-Shm

    add_library(switcherjacktoshm SHARED
        jack-client.cpp
        jack-to-shmdata.cpp
        )

    add_dependencies(switcherjacktoshm ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_jack check_jack.cpp)
    add_test(check_jack check_jack)

    # INSTALL

    install(TARGETS switchershmtojack switcherjacktoshm LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
