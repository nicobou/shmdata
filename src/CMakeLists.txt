include(CheckCXXCompilerFlag)

link_libraries(
    ${SWITCHER_LIBRARY}
)

# Switcher

cmake_dependent_option(WITH_SWITCHER "Switcher Command Line" ON
                       "PLUGIN_GSOAP" OFF)

add_feature_info("switcher" WITH_SWITCHER "Switcher Command Line")
if (WITH_SWITCHER)

    add_executable(switcher
        main.cpp
        )

    # INSTALL

    install(TARGETS switcher
        RUNTIME
        DESTINATION bin
        COMPONENT lib
        )

    install(TARGETS switcher
        RUNTIME
        DESTINATION bin
        COMPONENT dev
        )

    install(TARGETS switcher
        RUNTIME
        DESTINATION bin
        COMPONENT dbg
        )

    # TEST

    add_test(NAME check_switcher_command COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check_switcher.sh)

endif ()

# Switcher Control


cmake_dependent_option(WITH_SWITCHER_CTRL "Switcher Control" ON
                       "PLUGIN_GSOAP" OFF)
add_feature_info("switcher-ctrl" WITH_SWITCHER_CTRL "Switcher Control")
if (WITH_SWITCHER_CTRL)

    add_executable(switcher-ctrl
        switcher-ctrl.cpp
        )

    target_include_directories(switcher-ctrl PRIVATE
        soapclient-static
        )

    target_link_libraries(switcher-ctrl PRIVATE
        soapclient-static
        )

    add_dependencies(switcher-ctrl soapclient-static)

    # WARNINGS
    check_cxx_compiler_flag(-Wno-error=misleading-indentation CAN_IGNORE_MISLEADING_INDENTATION)
    if(CAN_IGNORE_MISLEADING_INDENTATION)
        add_definitions(-Wno-error=misleading-indentation)
    endif()
    
    # INSTALL

    install(TARGETS switcher-ctrl
        RUNTIME
        DESTINATION bin
        COMPONENT lib
        )

    install(TARGETS switcher-ctrl
        RUNTIME
        DESTINATION bin
        COMPONENT dev
        )

    install(TARGETS switcher-ctrl
        RUNTIME
        DESTINATION bin
        COMPONENT dbg
        )

    # TEST

    add_test(NAME check_switcher_ctrl_command COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check_switcher_ctrl.sh)

endif ()
