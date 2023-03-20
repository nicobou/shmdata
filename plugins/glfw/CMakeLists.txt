include(CheckCXXCompilerFlag)

set(PLUGIN_NAME "plugin-glfw")
set(PLUGIN_DESCRIPTION "GLFW Plugin")

option(PLUGIN_GLFW "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_GLFW "${PLUGIN_DESCRIPTION}")

if (PLUGIN_GLFW)

    # Externals, GLFW for now
    add_subdirectory(external)

    # sbt_image requires the following:
    check_cxx_compiler_flag(-Wno-error=misleading-indentation CAN_IGNORE_MISLEADING_INDENTATION)
    if(CAN_IGNORE_MISLEADING_INDENTATION)
        add_definitions(-Wno-error=misleading-indentation)
    endif()
    check_cxx_compiler_flag(-Wno-error=implicit-fallthrough CAN_IMPLICIT_FALLTHROUGH)
    if(CAN_IMPLICIT_FALLTHROUGH)
        add_definitions(-Wno-error=implicit-fallthrough)
    endif()
    check_cxx_compiler_flag(-Wno-error=shift-negative-value CAN_IGNORE_SHIFT_NEGATIVE_VALUE)
    if(CAN_IGNORE_SHIFT_NEGATIVE_VALUE)
        add_definitions(-Wno-error=shift-negative-value)
    endif()
    check_cxx_compiler_flag(-Wno-error=class-memaccess CAN_CLASS_MEMACCESS)
    if(CAN_CLASS_MEMACCESS)
        add_definitions(-Wno-error=class-memaccess)
    endif()

    # -Wno-error=uninitialized
    check_cxx_compiler_flag(-Wno-error=uninitialized SUPPORTS_UNINITIALIZED_FLAG)
    if (SUPPORTS_UNINITIALIZED_FLAG)
      add_definitions(-Wno-error=uninitialized)
    endif()
    # -Wno-error=unknown-pragmas
    check_cxx_compiler_flag(-Wno-error=unknown-pragmas SUPPORTS_UNKNOWN_PRAGMA_FLAG)
    if (SUPPORTS_UNKNOWN_PRAGMA_FLAG)
      add_definitions(-Wno-error=unknown-pragmas)
    endif ()
    # -Wno-error=maybe-uninitialized
    check_cxx_compiler_flag(-Wno-error=maybe-uninitialized SUPPORTS_MAY_UNINITIALIZED_FLAG)
    if (SUPPORTS_MAY_UNINITIALIZED_FLAG)
      add_definitions(-Wno-error=maybe-uninitialized)
    endif ()

    include_directories(
        ${GLFW_INCLUDE_DIRS}
    )

    link_libraries(
        glfw
        dl
    )

    if (OSX)
        link_libraries(
            "-framework OpenGL"
            "-framework Cocoa"
            "-framework IOKit"
            "-framework Corevideo"
        )
    else ()
        link_libraries(
            GL
            rt
            m
            X11
            Xrandr
            Xinerama
            Xxf86vm
            Xcursor
        )
    endif ()

    # IMGUI

    set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
    set(IMGUI_SOURCE_FILES ${IMGUI_SOURCE_DIR}/imgui.cpp ${IMGUI_SOURCE_DIR}/imgui_draw.cpp)

    include_directories(
        ${IMGUI_SOURCE_DIR}
    )

    # Plugin

    add_library(glfwvideo SHARED
        external/glad.c
        glfw-renderer.cpp
        glfwvideo.cpp
        ${IMGUI_SOURCE_FILES}
        )

    add_dependencies(glfwvideo ${SWITCHER_LIBRARY})
    add_dependencies(glfwvideo glfw)

    # TEST

    add_executable(check_glfw_video check_glfw_video.cpp)
    add_dependencies(check_glfw_video glfw)
    add_test(check_glfw_video check_glfw_video)
    add_test(NAME check_swcam_display COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check_swcam_display.sh)

    # INSTALL

    install(TARGETS glfwvideo LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/plugins/glfw/fonts DESTINATION ${DATADIR})
    install(TARGETS swcam-display DESTINATION bin
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
      GROUP_EXECUTE GROUP_READ
      WORLD_EXECUTE WORLD_READ)

endif ()
