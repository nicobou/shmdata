set(PLUGIN_NAME "plugin-werbtc")
set(PLUGIN_DESCRIPTION "WebRTC Plugin")

option(PLUGIN_WEBRTC "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_WEBRTC "${PLUGIN_DESCRIPTION}")

if (PLUGIN_WEBRTC)

  pkg_check_modules(LIBSOUP REQUIRED libsoup-2.4)
  if(LIBSOUP_FOUND)
    include_directories(${LIBSOUP_INCLUDE_DIRS})
    link_libraries(${LIBSOUP_LIBRARIES})
  endif()

  pkg_check_modules(LIBGSTWEBRTC REQUIRED gstreamer-webrtc-1.0)
  if(LIBGSTWEBRTC_FOUND)
    include_directories(${LIBGSTWEBRTC_INCLUDE_DIRS})
    link_libraries(${LIBGSTWEBRTC_LIBRARIES})
  endif()

  add_library(webrtc SHARED webrtc.cpp)

  add_executable(check_webrtc check_webrtc.cpp)
  add_test(check_webrtc check_webrtc)


  install(TARGETS webrtc LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)
endif ()
