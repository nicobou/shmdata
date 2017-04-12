# PLUGIN

find_path(VIDEODEV2 linux/videodev2.h)
set(ENABLED FALSE)
if (VIDEODEV2)
    set(ENABLED TRUE)
endif()

set(PLUGIN_NAME "plugin-v4l2")
set(PLUGIN_DESCRIPTION "Video for Linux 2 Plugin")

cmake_dependent_option(PLUGIN_V4L2 "${PLUGIN_DESCRIPTION}" ${ENABLED}
                       "ENABLE_GPL" OFF)
add_feature_info("${PLUGIN_NAME}" PLUGIN_V4L2 "${PLUGIN_DESCRIPTION}")

if (PLUGIN_V4L2)

    add_library(v4l2src SHARED
        v4l2src.cpp
        )

    add_dependencies(v4l2src ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_v4l2_plugin check_v4l2_plugin.cpp)
    add_test(check_v4l2_plugin check_v4l2_plugin)

    # INSTALL

    install(TARGETS v4l2src LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
