#pkg_check_modules(GMODULE REQUIRED gmodule-2.0)

option(WITH_OPTIONAL_PLUGINS "Optional Plugins" ON)
add_feature_info("plugins-optional" WITH_OPTIONAL_PLUGINS "Optional Plugins")

link_libraries(
    ${SWITCHER_LIBRARY}
    #${GMODULE_LIBRARIES}
)

include_directories(
    ${SWITCHER_DIR}
    #${GMODULE_INCLUDE_DIRS}
)

#add_compile_options(${GMODULE_CFLAGS})

# Required plugins
add_subdirectory(gsoap)
add_subdirectory(pjsip)
add_subdirectory(systemusage)

# Optional plugins
if (WITH_OPTIONAL_PLUGINS)
    add_subdirectory(avrecplay)
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
    add_subdirectory(syphon)
    add_subdirectory(timecode)
    add_subdirectory(v4l2)
    add_subdirectory(vnc)
    add_subdirectory(vrpn)
    add_subdirectory(watcher)
endif ()
