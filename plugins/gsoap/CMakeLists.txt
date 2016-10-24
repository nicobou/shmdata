# PLUGIN

pkg_check_modules(GSOAP gsoap++)

set(PLUGIN_NAME "plugin-gsoap")
set(PLUGIN_DESCRIPTION "GSoap Plugin")

option(PLUGIN_GSOAP "${PLUGIN_DESCRIPTION}" ${GSOAP_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_GSOAP "${PLUGIN_DESCRIPTION}")

if (PLUGIN_GSOAP)

    pkg_check_modules(GSOAP REQUIRED gsoap++)

    # Services

    set(GSOAP_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)

    add_custom_command(OUTPUT generated/webservices/soapC.cpp generated/webservices/soapcontrolProxy.cpp generated/webservices/soapcontrolService.cpp
        COMMAND ${CMAKE_COMMAND} -E make_directory generated/webservices
        COMMAND soapcpp2 -c++11 -i -d generated/webservices ${CMAKE_CURRENT_SOURCE_DIR}/ctrl-webservice-definition.hpp
        )

    add_custom_target(webservices DEPENDS generated/webservices/soapC.cpp)

    include_directories(
        ${GSOAP_INCLUDE_DIRS}
    )

    link_libraries(
        ${GSOAP_LIBRARIES}
    )

    # Server

    add_library(soapserver SHARED
        soap-ctrl-server.cpp
        generated/webservices/soapC.cpp
        generated/webservices/soapcontrolProxy.cpp
        generated/webservices/soapcontrolService.cpp
        )

    add_dependencies(soapserver ${SWITCHER_LIBRARY})

    target_include_directories(soapserver
        PUBLIC ${GSOAP_GEN_DIR}
        )

    add_dependencies(soapserver webservices)

    # Client
    set(CLIENT_SOURCES
        soap-ctrl-client.cpp
        generated/webservices/soapC.cpp
        generated/webservices/soapcontrolProxy.cpp
        )

    add_library(soapclient SHARED
        ${CLIENT_SOURCES}
        )

    add_dependencies(soapclient ${SWITCHER_LIBRARY})

    target_include_directories(soapclient
        PUBLIC ${GSOAP_GEN_DIR}
        )

    add_dependencies(soapclient webservices)

    # Client Static (for switcher-ctrl)

    add_library(soapclient-static STATIC
        ${CLIENT_SOURCES}
        )

    add_dependencies(soapclient-static ${SWITCHER_LIBRARY})

    target_include_directories(soapclient-static
        PUBLIC ${GSOAP_GEN_DIR}
        )

    add_dependencies(soapclient-static webservices)

    # TEST

    add_executable(check_gsoap_control check_gsoap_control.cpp)
    add_test(check_gsoap_control check_gsoap_control)

    # INSTALL

    install(TARGETS soapserver soapclient LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()