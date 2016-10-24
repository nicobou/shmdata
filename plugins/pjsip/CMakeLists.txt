set(PLUGIN_NAME "plugin-sip")
set(PLUGIN_DESCRIPTION "SIP Plugin")

option(PLUGIN_SIP "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_SIP "${PLUGIN_DESCRIPTION}")

if (PLUGIN_SIP)

    set(PJPROJECT_VERSION 2.5.1)
    set(PJ_URL http://www.pjsip.org/release/${PJPROJECT_VERSION}/pjproject-${PJPROJECT_VERSION}.tar.bz2)
    set(PJ_CHECKSUM 379562fe32ff279ba0ad8a6b81a3d0cb)
    set(PJ_BUILD_DIR ${CMAKE_BINARY_DIR}/pjproject)

    function(build_pjproject)
        file(MAKE_DIRECTORY ${PJ_BUILD_DIR} ${PJ_BUILD_DIR}/build)

        set(CMAKE_CONTENT "
        cmake_minimum_required(VERSION 3.5)
        include(ExternalProject)
        ExternalProject_Add(pjproject
            PREFIX pjproject
            URL ${PJ_URL}
            URL_HASH MD5=${PJ_CHECKSUM}
            PATCH_COMMAND patch -tp0 < ${CMAKE_CURRENT_SOURCE_DIR}/more_codecs.patch && ${CMAKE_CURRENT_SOURCE_DIR}/build_scripts/patch.sh
            BUILD_IN_SOURCE 1
            CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build_scripts/configure.sh
            BUILD_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build_scripts/build.sh
            INSTALL_COMMAND make install
            )
            ")

        file(WRITE ${PJ_BUILD_DIR}/CMakeLists.txt "${CMAKE_CONTENT}")

        execute_process(COMMAND ${CMAKE_COMMAND} ..
            WORKING_DIRECTORY ${PJ_BUILD_DIR}/build
            )
        execute_process(COMMAND ${CMAKE_COMMAND} --build .
            WORKING_DIRECTORY ${PJ_BUILD_DIR}/build
            )

    endfunction()

    build_pjproject()

    set(PJ_INSTALL_DIR ${PJ_BUILD_DIR}/build/pjproject/src/pjproject/install_dir)

    set(CMAKE_PREFIX_PATH ${PJ_INSTALL_DIR})
    pkg_check_modules(PJPROJECT REQUIRED libpjproject)

    link_directories(
        ${PJPROJECT_LIBRARY_DIRS}
    )

    # PLUGIN

    add_compile_options(${PJPROJECT_CFLAGS})

    include_directories(
        ${PJPROJECT_INCLUDE_DIRS}
    )

    link_libraries(
        ${PJPROJECT_LIBRARIES}
    )

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

    # TEST

    add_executable(check_pjsip check_pjsip.cpp)
    add_test(check_pjsip check_pjsip)

    # INSTALL

    install(TARGETS pjsip LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()