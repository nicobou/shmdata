set(PLUGIN_NAME "plugin-syphon")
set(PLUGIN_DESCRIPTION "Syphon Plugin")

option(PLUGIN_SYPHON "${PLUGIN_DESCRIPTION}" ${OSX})
add_feature_info("${PLUGIN_NAME}" PLUGIN_SYPHON "${PLUGIN_DESCRIPTION}")

if (PLUGIN_SYPHON)

    # Syphon

    include(ExternalProject)
    ExternalProject_Add(syphon
        GIT_REPOSITORY https://github.com/Syphon/Syphon-Framework.git
        UPDATE_COMMAND "" # Prevents rebuilds on every update
        CONFIGURE_COMMAND ""
        BUILD_IN_SOURCE 1
        BUILD_COMMAND xcodebuild -project Syphon.xcodeproj -configuration Release SYMROOT=../syphon-build DSTROOT=~
        INSTALL_COMMAND xcodebuild -project Syphon.xcodeproj -configuration Release install SYMROOT=../syphon-build DSTROOT=~
        )

    set(SYPHON_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/syphon-prefix/src/syphon-build/Release/Syphon.framework/Headers)
    set(SYPHON_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/syphon-prefix/src/syphon-build/Release/Syphon.framework)

    # Plugin

    include_directories(
        ${SYPHON_INCLUDE_DIRS}
    )

    link_libraries(
        "-framework OpenGL"
        "-framework Foundation"
        objc
    )

    add_library(syphonsrc SHARED
        syphonsrc.cpp
        syphonreader.mm
        )

    # WARNINGS
    add_definitions(-Wno-error=deprecated-declarations)
    add_definitions(-Wno-error=objc-method-access)
    add_definitions(-Wno-error=receiver-expr)
    
    target_compile_options(syphonsrc PUBLIC -F ${SYPHON_LIBRARIES})
    target_link_libraries(syphonsrc "-F ${SYPHON_LIBRARIES} -framework Syphon")
    add_dependencies(syphonsrc ${SWITCHER_LIBRARY})
    add_dependencies(syphonsrc syphon)

    # TEST

    add_executable(check_syphonsrc_plugin check_syphonsrc_plugin.cpp)
    target_compile_options(check_syphonsrc_plugin PUBLIC -F ${SYPHON_LIBRARIES})
    target_link_libraries(check_syphonsrc_plugin "-F ${SYPHON_LIBRARIES} -framework Syphon")
    add_dependencies(check_syphonsrc_plugin syphon)
    add_test(check_syphonsrc_plugin check_syphonsrc_plugin)

    # INSTALL

    install(TARGETS syphonsrc LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
