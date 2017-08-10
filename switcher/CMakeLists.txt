pkg_check_modules(GMODULE REQUIRED gmodule-2.0)

add_library(${SWITCHER_LIBRARY} SHARED
    audio-test-source.cpp
    bundle-description-parser.cpp
    bundle.cpp
    categorizable.cpp
    color.cpp
    counter-map.cpp
    decodebin-to-shmdata.cpp
    documentation-registry.cpp
    dummy-sink.cpp
    empty-quiddity.cpp
    external-shmdata-writer.cpp
    file-utils.cpp
    fraction.cpp
    g-source-wrapper.cpp
    glibmainloop.cpp
    gprop-to-prop.cpp
    gst-audio-codec.cpp
    gst-audio-encoder.cpp
    gst-decodebin.cpp
    gst-pipe.cpp
    gst-pipeliner.cpp
    gst-pixel-format-converter.cpp
    gst-rtppayloader-finder.cpp
    gst-shmdata-subscriber.cpp
    gst-shmdata-to-cb.cpp
    gst-utils.cpp
    gst-video-codec.cpp
    gst-video-converter.cpp
    gst-video-encoder.cpp
    gst-video-timelapse.cpp
    http-sdp-dec.cpp
    information-tree-basic-serializer.cpp
    information-tree-json.cpp
    information-tree.cpp
    invocation-spec.cpp
    json-builder.cpp
    method.cpp
    net-utils.cpp
    plugin-loader.cpp
    property-container.cpp
    property-mapper.cpp
    property.cpp
    quiddity-basic-test.cpp
    quiddity-documentation.cpp
    quiddity-container.cpp
    switcher-wrapper.cpp
    switcher.cpp
    quiddity.cpp
    rtp-destination.cpp
    rtp-receiver.cpp
    rtp-sender.cpp
    rtp-session.cpp
    rtp-session2.cpp
    safe-bool-idiom.cpp
    sdp-utils.cpp
    serialize-string.cpp
    shm-delay.cpp
    shmdata-connector.cpp
    shmdata-follower.cpp
    shmdata-stat.cpp
    shmdata-utils.cpp
    shmdata-writer.cpp
    sig.cpp
    signal-container.cpp
    startable-quiddity.cpp
    string-utils.cpp
    timelapse.cpp
    type-name-registry.cpp
    unique-gst-element.cpp
    uridecodebin.cpp
    video-test-source.cpp
    )

set(HEADER_INCLUDES
    base-logger.hpp
    bool-log.hpp
    abstract-factory.hpp
    abstract-factory_spec.hpp
    any.hpp
    audio-test-source.hpp
    bundle.hpp
    bundle-description-parser.hpp
    categorizable.hpp
    color.hpp
    console-logger.hpp
    counter-map.hpp
    creator.hpp
    decodebin-to-shmdata.hpp
    documentation-registry.hpp
    dummy-sink.hpp
    empty-quiddity.hpp
    exec-loop.hpp
    external-shmdata-writer.hpp
    file-utils.hpp
    fraction.hpp
    g-source-wrapper.hpp
    glibmainloop.hpp
    gprop-to-prop.hpp
    group.hpp
    gst-audio-codec.hpp
    gst-audio-encoder.hpp
    gst-decodebin.hpp
    gst-pipe.hpp
    gst-pipeliner.hpp
    gst-pixel-format-converter.hpp
    gst-rtppayloader-finder.hpp
    gst-shmdata-subscriber.hpp
    gst-shmdata-to-cb.hpp
    gst-utils.hpp
    gst-video-codec.hpp
    gst-video-timelapse.hpp
    gst-video-converter.hpp
    gst-video-encoder.hpp
    http-sdp-dec.hpp
    information-tree-basic-serializer.hpp
    information-tree-json.hpp
    information-tree.hpp
    is-specialization-of.hpp
    json-builder.hpp
    logged.hpp
    make-consultable.hpp
    method.hpp
    net-utils.hpp
    periodic-task.hpp
    plugin-loader.hpp
    property-container.hpp
    property-internal-types.hpp
    property-mapper.hpp
    property-specification.hpp
    property.hpp
    quiddity-basic-test.hpp
    quiddity-configuration.hpp
    invocation-spec.hpp
    quiddity-documentation.hpp
    quiddity-container.hpp
    switcher-wrapper.hpp
    switcher.hpp
    quiddity.hpp
    rtp-destination.hpp
    rtp-receiver.hpp
    rtp-sender.hpp
    rtp-session.hpp
    rtp-session2.hpp
    safe-bool-idiom.hpp
    scope-exit.hpp
    sdp-utils.hpp
    selection.hpp
    serialize-string.hpp
    shm-delay.hpp
    shmdata-connector.hpp
    shmdata-follower.hpp
    shmdata-stat.hpp
    shmdata-switcher-logger.hpp
    shmdata-utils.hpp
    shmdata-writer.hpp
    sig.hpp
    signal-container.hpp
    silent-logger.hpp
    startable-quiddity.hpp
    string-utils.hpp
    templated-sequence.hpp
    threaded-wrapper.hpp
    timelapse.hpp
    type-name-registry.hpp
    unique-gst-element.hpp
    uridecodebin.hpp
    video-test-source.hpp
    )

if (OSX)
    target_link_libraries(${SWITCHER_LIBRARY} PRIVATE resolv)
endif()

set_target_properties(${SWITCHER_LIBRARY} PROPERTIES VERSION ${SWITCHER_VERSION_STRING} SOVERSION ${SWITCHER_VERSION_MAJOR})
target_compile_options(${SWITCHER_LIBRARY} PUBLIC -DG_LOG_DOMAIN="switcher")
target_compile_options(${SWITCHER_LIBRARY} PUBLIC ${GMODULE_CFLAGS})

target_link_libraries(${SWITCHER_LIBRARY}
    PRIVATE ${COMMON_LIBRARIES}
    PUBLIC ${GMODULE_LIBRARIES}
    )

target_include_directories(${SWITCHER_LIBRARY}
    PRIVATE ${COMMON_INCLUDE_DIRS}
    PUBLIC ${GMODULE_INCLUDE_DIRS}
    )


# Pkg-config
if (UNIX)
    include(FindPkgConfig QUIET)
    if (PKG_CONFIG_FOUND)
        configure_file("switcher.pc.in" "switcher-${SWITCHER_API_VERSION}.pc" @ONLY)
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/switcher-${SWITCHER_API_VERSION}.pc"
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
    endif ()
endif ()

install(TARGETS ${SWITCHER_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT libraries)

install(FILES ${HEADER_INCLUDES}
    DESTINATION include/switcher
    COMPONENT headers
    )
