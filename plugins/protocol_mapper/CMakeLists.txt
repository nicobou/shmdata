# PLUGIN

pkg_check_modules(LIBCURL libcurl)
pkg_check_modules(LIBLO liblo)

set(ENABLED FALSE)
if (LIBCURL_FOUND AND LIBLO_FOUND)
    set(ENABLED TRUE)
endif ()

set(PLUGIN_NAME "plugin-protocol-mapper")
set(PLUGIN_DESCRIPTION "Protocol Mapper Plugin")

option(PLUGIN_PROTOCOL_MAPPER "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_PULSE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_PROTOCOL_MAPPER)

    include_directories(
        ${LIBLO_INCLUDE_DIRS}
        ${LIBCURL_INCLUDE_DIRS}
    )

    link_libraries(
        ${LIBLO_LIBRARIES}
        ${LIBCURL_LIBRARIES}
    )

    add_library(protocol_mapper SHARED
        protocol-curl.cpp
        protocol-mapper.cpp
        protocol-osc.cpp
        protocol-reader.cpp
    )
    add_dependencies(protocol_mapper ${SWITCHER_LIBRARY})

    # Tests
    configure_file(protocol-curl.json protocol-curl.json COPYONLY)
    configure_file(protocol-osc.json protocol-osc.json COPYONLY)
    add_executable(check_protocol_curl check_curl.cpp)
    add_test(check_protocol_curl check_protocol_curl)
    add_executable(check_protocol_osc check_osc.cpp)
    add_test(check_protocol_osc check_protocol_osc)


    install(TARGETS protocol_mapper LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()