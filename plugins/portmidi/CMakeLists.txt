# PLUGIN

find_library(PORTMIDI portmidi)

set(ENABLED FALSE)
if (PORTMIDI)
    set(ENABLED TRUE)
endif ()

set(PLUGIN_NAME "plugin-midi")
set(PLUGIN_DESCRIPTION "MIDI Plugin")

option(PLUGIN_MIDI "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_MIDI "${PLUGIN_DESCRIPTION}")

if (PLUGIN_MIDI)

    if (NOT PORTMIDI)
        message(FATAL_ERROR "portmidi not found")
    endif ()

    link_libraries(
        ${PORTMIDI}
    )

    # SRC

    add_library(portmidisrc SHARED
        portmidi-devices.cpp
        portmidi-source.cpp
        )

    add_dependencies(portmidisrc ${SWITCHER_LIBRARY})

    # SINK

    add_library(portmidisink SHARED
        portmidi-devices.cpp
        portmidi-sink.cpp
        )

    add_dependencies(portmidisink ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_midi_plugins check_midi_plugins.cpp)
    add_test(check_midi_plugins check_midi_plugins)

    # INSTALL

    install(TARGETS portmidisrc portmidisink LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()