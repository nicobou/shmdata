
option(WITH_OPTIONAL_PLUGINS "Optional Plugins" ON)
add_feature_info("plugins-optional" WITH_OPTIONAL_PLUGINS "Optional Plugins")

link_libraries(
    ${SWITCHER_LIBRARY}
)

include_directories(
    ${SWITCHER_DIR}
)

# Required plugins
add_subdirectory(gsoap)
add_subdirectory(pjsip)
add_subdirectory(systemusage)

# Optional plugins
if (WITH_OPTIONAL_PLUGINS)
    add_subdirectory(avrecplay)
    add_subdirectory(crashtest)
    add_subdirectory(cropper)
    add_subdirectory(example)
    add_subdirectory(executor)
    add_subdirectory(glfw)
    add_subdirectory(jack)
    add_subdirectory(ladspa)
    add_subdirectory(nvenc)
    add_subdirectory(osc)
    add_subdirectory(portmidi)
    add_subdirectory(protocol_mapper)
    add_subdirectory(pulse)
    add_subdirectory(rtmp)
    add_subdirectory(samplerate)
    add_subdirectory(timecode)
    add_subdirectory(v4l2)
    add_subdirectory(video_snapshot)
    add_subdirectory(vnc)
    add_subdirectory(vrpn)
    add_subdirectory(watcher)
    add_subdirectory(webrtc)
endif ()
