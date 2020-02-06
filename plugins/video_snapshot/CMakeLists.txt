# PLUGIN

set(PLUGIN_NAME "plugin-video-snapshot")
set(PLUGIN_DESCRIPTION "Plugin for video snapshot")

option(PLUGIN_VIDEO_SNAPSHOT "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_VIDEO_SNAPSHOT "${PLUGIN_DESCRIPTION}")

if (PLUGIN_VIDEO_SNAPSHOT)

    # PLUGIN
  
    add_library(video_snapshot SHARED
        video_snapshot.cpp
    )
    add_dependencies(video_snapshot ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_video_snapshot check_video_snapshot.cpp)
    add_test(check_video_snapshot check_video_snapshot)

    # INSTALL

    install(TARGETS video_snapshot LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
