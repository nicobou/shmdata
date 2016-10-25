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
        INSTALL_COMMAND ""
        )

    set(GLFW_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/glfw-prefix/src/glfw/include)
    set(GLFW_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/glfw-prefix/src/glfw-build/src/${CMAKE_STATIC_LIBRARY_PREFIX}glfw3${CMAKE_STATIC_LIBRARY_SUFFIX})

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
        )
    else ()
	# The following are needed because we statically link libglfw.
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

    # Plugin

    add_library(glfwvideo SHARED
        glad.c
        glfw-renderer.cpp
        glfwvideo.cpp
        )

    add_dependencies(glfwvideo ${SWITCHER_LIBRARY})
    add_dependencies(glfwvideo glfw)

    # TEST

    add_executable(check_glfw_video check_glfw_video.cpp)
    add_dependencies(check_glfw_video glfw)
    add_test(check_glfw_video check_glfw_video)

    # INSTALL

    install(TARGETS glfwvideo LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
