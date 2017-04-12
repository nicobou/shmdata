include(CheckCXXCompilerFlag)

set(PLUGIN_NAME "plugin-glfw")
set(PLUGIN_DESCRIPTION "GLFW Plugin")

option(PLUGIN_GLFW "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_GLFW "${PLUGIN_DESCRIPTION}")

if (PLUGIN_GLFW)

    # GLFW

    include(ExternalProject)
    ExternalProject_Add(glfw
        URL https://github.com/glfw/glfw/archive/3.2.1.tar.gz
        CMAKE_ARGS -DBUILD_SHARED_LIBS=FALSE
        CMAKE_ARGS -DGLFW_BUILD_EXAMPLES=FALSE
        CMAKE_ARGS -DGLFW_BUILD_TESTS=FALSE
        CMAKE_ARGS -DGLFW_BUILD_DOCS=FALSE
        CMAKE_ARGS -DGLFW_INSTALL=FALSE
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        INSTALL_COMMAND ""
        )

    set(GLFW_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/glfw-prefix/src/glfw/include)
    set(GLFW_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/glfw-prefix/src/glfw-build/src/${CMAKE_STATIC_LIBRARY_PREFIX}glfw3${CMAKE_STATIC_LIBRARY_SUFFIX})

    # sbt_image requires the following:
    check_cxx_compiler_flag(-Wno-error=misleading-indentation CAN_IGNORE_MISLEADING_INDENTATION)
    if(CAN_IGNORE_MISLEADING_INDENTATION)
        add_definitions(-Wno-error=misleading-indentation)
    endif()

    check_cxx_compiler_flag(-Wno-error=shift-negative-value CAN_IGNORE_SHIFT_NEGATIVE_VALUE)
    if(CAN_IGNORE_SHIFT_NEGATIVE_VALUE)
        add_definitions(-Wno-error=shift-negative-value)
    endif()

    if (OSX)
        add_definitions(-Wno-error=uninitialized)
        add_definitions(-Wno-error=unknown-pragmas)
    else ()
        add_definitions(-Wno-error=maybe-uninitialized)
    endif ()

    include_directories(
        ${GLFW_INCLUDE_DIRS}
    )

    link_libraries(
        ${GLFW_LIBRARIES}
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

    set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/imgui-prefix/src/imgui)
    set(IMGUI_SOURCE_FILES ${IMGUI_SOURCE_DIR}/imgui.cpp ${IMGUI_SOURCE_DIR}/imgui_draw.cpp)

    ExternalProject_Add(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.49
        DOWNLOAD_DIR ${IMGUI_SOURCE_DIR}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        )

    include_directories(
        ${IMGUI_SOURCE_DIR}
    )

    # Plugin

    add_custom_command(OUTPUT ${IMGUI_SOURCE_FILES} DEPENDS imgui COMMAND "")

    add_library(glfwvideo SHARED
        glad.c
        glfw-renderer.cpp
        glfwvideo.cpp
        ${IMGUI_SOURCE_FILES}
        )

    add_dependencies(glfwvideo ${SWITCHER_LIBRARY})
    add_dependencies(glfwvideo glfw)
    add_dependencies(glfwvideo imgui)

    # TEST

    add_executable(check_glfw_video check_glfw_video.cpp)
    add_dependencies(check_glfw_video glfw)
    add_test(check_glfw_video check_glfw_video)

    # INSTALL

    install(TARGETS glfwvideo LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/plugins/glfw/fonts DESTINATION ${DATADIR})

endif ()
