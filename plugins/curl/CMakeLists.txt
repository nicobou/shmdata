# PLUGIN

set(PLUGIN_NAME "plugin-curl")
set(PLUGIN_DESCRIPTION "Curl Plugin")

option(PLUGIN_CURL "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_CURL "${PLUGIN_DESCRIPTION}")

if (PLUGIN_CURL)

    pkg_check_modules(CURL REQUIRED libcurl)

    add_compile_options(${CURL_CFLAGS})

    include_directories(
        ${CURL_INCLUDE_DIRS}
    )

    link_libraries(
        ${CURL_LIBRARIES}
    )

    # Curl HTTP get plugin

    add_library(http_get_service SHARED
        http-get-service-plugin.cpp
        )

    # TEST

    add_executable(check_curl check_curl.cpp)
    add_test(check_curl check_curl)

    # INSTALL

    install(TARGETS http_get_service LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
