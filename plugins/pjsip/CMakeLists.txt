set(PLUGIN_NAME "plugin-sip")
set(PLUGIN_DESCRIPTION "SIP Plugin")

cmake_dependent_option(PLUGIN_SIP "${PLUGIN_DESCRIPTION}" ON
                       "ENABLE_GPL" OFF)

add_feature_info("${PLUGIN_NAME}" PLUGIN_SIP "${PLUGIN_DESCRIPTION}")

if (PLUGIN_SIP)

    include(ExternalProject)

    set(PJPROJECT_VERSION 2.10)

    set(PJPROJECT_CFLAGS
        -DPJ_ICE_ST_MAX_CAND=512
        -DPJ_ICE_MAX_CHECKS=512
        -DPJ_ICE_COMP_BITS=8
        -DPJ_ICE_MAX_CAND=1024
        -DPJ_ICE_CAND_TYPE_PREF_BITS=16
        -DPJ_IOQUEUE_MAX_EVENTS_IN_SINGLE_POLL=512
        -DPJSIP_MAX_PKT_LEN=8000
        -DPJSIP_TD_TIMEOUT=5000
	-DPJ_AUTOCONF=1
        -DPJ_ENABLE_EXTRA_CHECK=1
	-DNDEBUG
        )

    # include/pj/compat/os_darwinos.h:107:9: error: 'PJ_IOQUEUE_MAX_HANDLES' macro redefined [-Werror,-Wmacro-redefined]
    # Default in OSX is 1024
    if(NOT OSX)
        list(APPEND PJPROJECT_CFLAGS -DPJ_IOQUEUE_MAX_HANDLES=512)
    endif()

    set(PJPROJECT_CONFIGURE_OPTIONS
        --prefix ${CMAKE_CURRENT_BINARY_DIR}/pjproject-install
        --disable-video
        --disable-sound
        --disable-g711-codec
        --disable-g7221-codec
        --disable-g722-codec
        --disable-gsm-codec
        --disable-ilbc-codec
        --disable-l16-codec
        --disable-speex-aec
        --disable-speex-codec
        --disable-libwebrtc
	--disable-ssl
        )

    # Convert CMake lists into space delimited strings for the command line
    string(REPLACE ";" " " PJPROJECT_CFLAGS_STR "${PJPROJECT_CFLAGS}")
    string(REPLACE ";" " " PJPROJECT_CONFIGURE_OPTIONS_STR "${PJPROJECT_CONFIGURE_OPTIONS}")

    # Generate the various build script
    configure_file("build_scripts/patch.sh.in" "patch.sh")
    configure_file("build_scripts/configure.sh.in" "configure.sh")
    configure_file("build_scripts/build.sh.in" "build.sh")

    # Configure the external project
    ExternalProject_Add(pjproject
        URL ${CMAKE_CURRENT_SOURCE_DIR}/external/pjsip
        PATCH_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/patch.sh
        BUILD_IN_SOURCE 1
        CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/configure.sh
        BUILD_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/build.sh
        INSTALL_COMMAND make install
    )

    # We have to manually retrieve the following locations and libraries since we don't have access
    # to the libpjproject.pc at configure time. the *-switcher suffix comes from the patch.sh script
    # where we replace the default value that is the system/os name in order to retrieve the right lib
    # no matter the system we're on.
    set(PJPROJECT_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/pjproject-install/include)
    set(PJPROJECT_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}/pjproject-install/lib)
    set(PJPROJECT_LIBRARIES
        pjsua2-switcher
        stdc++
        pjsua-switcher
        pjsip-ua-switcher
        pjsip-simple-switcher
        pjsip-switcher
        pjmedia-codec-switcher
        pjmedia-switcher
        pjmedia-videodev-switcher
        pjmedia-audiodev-switcher
        pjmedia-switcher
        pjnath-switcher
        pjlib-util-switcher
        srtp-switcher
        resample-switcher
        pj-switcher
        ssl
        crypto
        m
        pthread
        uuid
        )

    # PLUGIN

    add_compile_options(${PJPROJECT_CFLAGS})

    link_directories(
        ${PJPROJECT_LIBRARY_DIRS}
    )

    include_directories(
        ${PJPROJECT_INCLUDE_DIRS}
    )

    link_libraries(
        ${PJPROJECT_LIBRARIES}
    )

    if(OSX)
        link_libraries(
            "-framework Foundation"
            "-framework AppKit"
        )
    endif()

    add_library(pjsip SHARED
        pj-call.cpp
        pj-codec-utils.cpp
        pj-codec.cpp
        pj-ice-stream-trans.cpp
        pj-media-endpt.cpp
        pj-presence.cpp
        pj-sip-plugin.cpp
        pj-sip.cpp
        pj-stun-turn.cpp
        pj-whitelist.cpp
        )

    add_dependencies(pjsip ${SWITCHER_LIBRARY})
    add_dependencies(pjsip pjproject)

    # TEST

    add_executable(check_pjsip check_pjsip.cpp)
    add_dependencies(check_pjsip pjproject)
    add_test(check_pjsip check_pjsip)

    # INSTALL

    install(TARGETS pjsip LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)
    install(TARGETS swsip-reflector DESTINATION bin
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
      GROUP_EXECUTE GROUP_READ
      WORLD_EXECUTE WORLD_READ)

endif ()
