# PLUGIN

pkg_check_modules(JACK jack)

set(PLUGIN_NAME "plugin-jack")
set(PLUGIN_DESCRIPTION "Jack Plugin")

option(PLUGIN_JACK "${PLUGIN_DESCRIPTION}" ${JACK_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_JACK "${PLUGIN_DESCRIPTION}")

if (PLUGIN_JACK)

    pkg_check_modules(JACK REQUIRED jack)

    # jack/types.h requires the following:
    check_cxx_compiler_flag(-Wno-error=deprecated-register CAN_IGNORE_DEPRECATED_REGISTER)
    if(CAN_IGNORE_DEPRECATED_REGISTER)
        add_definitions(-Wno-error=deprecated-register)
    endif()
    
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

    # Jack Server

    link_libraries(
      ${JACK_LIBRARIES}
      -ljackserver
    )

    add_library(switcherjackserver SHARED
        jack-server-quid.cpp
        jack-server.cpp
        )

    add_dependencies(switcherjackserver ${SWITCHER_LIBRARY})


    # TEST

    add_executable(check_jack check_jack.cpp)
    add_test(check_jack check_jack)
    add_test(check_quid-conf-and-two-jack-servers check-quid-conf-and-two-jack-servers.py)
    add_test(NAME check_shmdata2jack COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check_shmdata2jack.sh)

    # INSTALL

    install(TARGETS switchershmtojack switcherjacktoshm switcherjackserver LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)
    install(TARGETS shmdata2jack DESTINATION bin
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
      GROUP_EXECUTE GROUP_READ
      WORLD_EXECUTE WORLD_READ)

endif ()
