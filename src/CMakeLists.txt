link_libraries(
    ${SWITCHER_LIBRARY}
)

# Switcher

cmake_dependent_option(WITH_SWITCHER "Switcher Command Line" ON
                       "PLUGIN_GSOAP" OFF)

add_feature_info("switcher" WITH_SWITCHER "Switcher Command Line")
if (WITH_SWITCHER)

    pkg_check_modules(GTK gtk+-3.0)

    # Replaces config.h in some cases
    add_definitions(
        -DLIBSWITCHER_API_VERSION="${SWITCHER_API_VERSION}"
        -DPACKAGE_NAME="${PACKAGE_NAME}"
        -DVERSION="${SWITCHER_VERSION_STRING}"
    )

    add_executable(switcher
        main.cpp
        )

    if (GTK_FOUND)
        add_definitions(-DHAVE_GTK)
        target_include_directories(switcher PRIVATE
            ${GTK_INCLUDE_DIRS}
            )
        target_link_libraries(switcher PRIVATE
            ${GTK_LIBRARIES}
            )
    endif ()

    # INSTALL

    install(TARGETS switcher
        RUNTIME
        DESTINATION bin
        COMPONENT applications
        )

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
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 6.0)
      add_definitions(-Wno-error=misleading-indentation)
    endif()
    
    # INSTALL

    install(TARGETS switcher-ctrl
        RUNTIME
        DESTINATION bin
        COMPONENT applications
        )

endif ()
