include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wno-error=cast-function-type CAN_CAST_FUNCTION_TYPE)
if(CAN_CAST_FUNCTION_TYPE)
  add_definitions(-Wno-error=cast-function-type)
endif()


pkg_check_modules(GMODULE REQUIRED gmodule-2.0)

add_library(${SWITCHER_LIBRARY} SHARED
  configuration/configuration.cpp
  gst/decodebin-to-shmdata.cpp
  gst/g-source-wrapper.cpp
  gst/glibmainloop.cpp
  gst/gst-audio-codec.cpp
  gst/gst-initialized.cpp
  gst/gst-pipe.cpp
  gst/gst-pipeliner.cpp
  gst/gst-pixel-format-converter.cpp
  gst/gst-rtppayloader-finder.cpp
  gst/gst-shmdata-to-cb.cpp
  gst/gst-utils.cpp
  gst/gst-video-codec.cpp
  gst/gst-video-timelapse.cpp
  gst/rtp-receiver.cpp
  gst/rtp-sender.cpp
  gst/rtp-session.cpp
  gst/sdp-utils.cpp
  gst/unique-gst-element.cpp
  infotree/information-tree-basic-serializer.cpp
  infotree/information-tree-json.cpp
  infotree/information-tree.cpp
  quiddities/audio-test-source.cpp
  quiddities/dummy-sink.cpp
  quiddities/empty-quiddity.cpp
  quiddities/external-shmdata-writer.cpp
  quiddities/file-decoder.cpp
  quiddities/gst-audio-encoder.cpp
  quiddities/gst-decodebin.cpp
  quiddities/gst-video-converter.cpp
  quiddities/gst-video-encoder.cpp
  quiddities/http-sdp-dec.cpp
  quiddities/shm-delay.cpp
  quiddities/timelapse.cpp
  quiddities/uridecodebin.cpp
  quiddities/video-test-source.cpp
  quiddity/bundle/bundle-description-parser.cpp
  quiddity/bundle/bundle.cpp
  quiddity/documentation-registry.cpp
  quiddity/method/method-container.cpp
  quiddity/plugin-loader.cpp
  quiddity/property/color.cpp
  quiddity/property/fraction.cpp
  quiddity/property/gprop-to-prop.cpp
  quiddity/property/property-container.cpp
  quiddity/property/property.cpp
  quiddity/quiddity-basic-test.cpp
  quiddity/quiddity-class-printer.cpp
  quiddity/quiddity-container.cpp
  quiddity/quiddity-documentation.cpp
  quiddity/quiddity-factory.cpp
  quiddity/quiddity-qrox.cpp
  quiddity/quiddity.cpp
  quiddity/signal/sig.cpp
  quiddity/signal/signal-container.cpp
  quiddity/startable-quiddity.cpp
  shmdata/caps/audio-caps.cpp
  shmdata/caps/utils.cpp
  shmdata/gst-shm-tree-updater.cpp
  shmdata/gst-shmdata-subscriber.cpp
  shmdata/shmdata-connector.cpp
  shmdata/shmdata-follower.cpp
  shmdata/shmdata-stat.cpp
  shmdata/shmdata-writer.cpp
  switcher.cpp
  utils/bool-any.cpp
  utils/bool-log.cpp
  utils/counter-map.cpp
  utils/file-utils.cpp
  utils/net-utils.cpp
  utils/safe-bool-idiom.cpp
  utils/serialize-string.cpp
  utils/string-utils.cpp
  utils/type-name-registry.cpp
  )

set(HEADER_DIR_INCLUDES
  configuration
  gst
  infotree
  logger
  quiddities
  quiddity
  shmdata
  utils
  )

set(HEADER_INCLUDES
  switcher.hpp
  )

if (OSX)
    target_link_libraries(${SWITCHER_LIBRARY} PRIVATE resolv)
endif()

set_target_properties(${SWITCHER_LIBRARY} PROPERTIES VERSION ${SWITCHER_VERSION_STRING} SOVERSION ${SWITCHER_VERSION_MAJOR})
target_compile_options(${SWITCHER_LIBRARY} PUBLIC -DSWITCHER_DEFAULT_PLUGIN_DIR="${CMAKE_INSTALL_PREFIX}/${SWITCHER_LIBRARY}/plugins" -DSWITCHER_VERSION_STRING="${SWITCHER_VERSION_STRING}" )
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

install(DIRECTORY ${HEADER_DIR_INCLUDES}
    DESTINATION include/switcher-${SWITCHER_API_VERSION}/switcher
    COMPONENT headers
    )

install(FILES ${HEADER_INCLUDES}
    DESTINATION include/switcher-${SWITCHER_API_VERSION}/switcher
    COMPONENT headers
    )
