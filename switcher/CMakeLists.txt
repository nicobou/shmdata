include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wno-error=cast-function-type CAN_CAST_FUNCTION_TYPE)
if(CAN_CAST_FUNCTION_TYPE)
  add_definitions(-Wno-error=cast-function-type)
endif()

add_library(${SWITCHER_LIBRARY} SHARED
  configuration/configurable.cpp
  configuration/configuration.cpp
  gst/audio-codec.cpp
  gst/decodebin-to-shmdata.cpp
  gst/g-source-wrapper.cpp
  gst/glibmainloop.cpp
  gst/initialized.cpp
  gst/pipe.cpp
  gst/pipeliner.cpp
  gst/pixel-format-converter.cpp
  gst/rtp-receiver.cpp
  gst/rtp-sender.cpp
  gst/rtp-session.cpp
  gst/rtppayloader-finder.cpp
  gst/sdp-utils.cpp
  gst/shmdata-to-cb.cpp
  gst/unique-gst-element.cpp
  gst/utils.cpp
  gst/video-codec.cpp
  gst/video-timelapse.cpp
  infotree/key-val-serializer.cpp
  infotree/json-serializer.cpp
  infotree/information-tree.cpp
  logger/log.cpp
  logger/logger.cpp
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
  quiddity/basic-test.cpp
  quiddity/bundle/bundle.cpp
  quiddity/bundle/description-parser.cpp
  quiddity/kind-printer.cpp
  quiddity/claw/claw.cpp
  quiddity/claw/connection-spec.cpp
  quiddity/container.cpp
  quiddity/doc.cpp
  quiddity/documentation-registry.cpp
  quiddity/factory.cpp
  quiddity/method/mbag.cpp
  quiddity/plugin-loader.cpp
  quiddity/property/color.cpp
  quiddity/property/fraction.cpp
  quiddity/property/gprop-to-prop.cpp
  quiddity/property/pbag.cpp
  quiddity/property/property.cpp
  quiddity/qrox.cpp
  quiddity/quiddity.cpp
  quiddity/signal/sbag.cpp
  quiddity/signal/sig.cpp
  quiddity/startable.cpp
  session/session.cpp
  shmdata/caps/audio-caps.cpp
  shmdata/caps/utils.cpp
  shmdata/follower.cpp
  shmdata/gst-subscriber.cpp
  shmdata/gst-tree-updater.cpp
  shmdata/stat.cpp
  shmdata/writer.cpp
  switcher.cpp
  utils/bool-any.cpp
  utils/bool-log.cpp
  utils/counter-map.cpp
  utils/file-utils.cpp
  utils/ids.cpp
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
  session
  shmdata
  utils
  )

if (OSX)
    target_link_libraries(${SWITCHER_LIBRARY} PRIVATE resolv)
endif()

set_target_properties(${SWITCHER_LIBRARY} PROPERTIES VERSION ${SWITCHER_VERSION_STRING} SOVERSION ${SWITCHER_VERSION_MAJOR})
target_compile_options(${SWITCHER_LIBRARY} PUBLIC -DSWITCHER_DEFAULT_PLUGIN_DIR="${CMAKE_INSTALL_PREFIX}/${SWITCHER_LIBRARY}/plugins" -DSWITCHER_VERSION_STRING="${SWITCHER_VERSION_STRING}" )

target_link_libraries(${SWITCHER_LIBRARY}
    PRIVATE ${COMMON_LIBRARIES}
    PUBLIC -Wl,--export-dynamic
    )

target_include_directories(${SWITCHER_LIBRARY}
    PRIVATE ${COMMON_INCLUDE_DIRS}
    )


# Pkg-config
if (UNIX)
  include(FindPkgConfig QUIET)
  if (PKG_CONFIG_FOUND)
    configure_file("switcher.pc.in" "switcher-${SWITCHER_API_VERSION}.pc" @ONLY)
    # Install Pkg-config file for the 'lib' package
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/switcher-${SWITCHER_API_VERSION}.pc"
      COMPONENT lib
      DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
    # Install Pkg-config file for the 'dev' package
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/switcher-${SWITCHER_API_VERSION}.pc"
      COMPONENT dev
      DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
    # Install Pkg-config file for the 'dbg' package
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/switcher-${SWITCHER_API_VERSION}.pc"
      COMPONENT dbg
      DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
  endif ()
endif ()

install(TARGETS ${SWITCHER_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT lib)

install(TARGETS ${SWITCHER_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT dev)

install(TARGETS ${SWITCHER_LIBRARY}
    LIBRARY DESTINATION lib
    COMPONENT dbg)

install(DIRECTORY .
    DESTINATION include/switcher-${SWITCHER_API_VERSION}/switcher
    COMPONENT dev
    FILES_MATCHING # install only matched files
    PATTERN "*.hpp" # select header files
    )

  install(DIRECTORY .
    DESTINATION include/switcher-${SWITCHER_API_VERSION}/switcher
    COMPONENT dbg
    FILES_MATCHING # install only matched files
    PATTERN "*.hpp" # select header files
    )

  
