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
    add_test(NAME check_shmshot COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check_shmshot.sh)

    # INSTALL

    install(TARGETS video_snapshot LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)
    install(TARGETS shmshot DESTINATION bin
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
      GROUP_EXECUTE GROUP_READ
      WORLD_EXECUTE WORLD_READ)

endif ()
