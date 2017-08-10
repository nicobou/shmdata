# PLUGIN

pkg_check_modules(GSOAP gsoap++)

set(PLUGIN_NAME "plugin-gsoap")
set(PLUGIN_DESCRIPTION "GSoap Plugin")

cmake_dependent_option(PLUGIN_GSOAP "${PLUGIN_DESCRIPTION}" ${GSOAP_FOUND}
                       "ENABLE_GPL" OFF)

add_feature_info("${PLUGIN_NAME}" PLUGIN_GSOAP "${PLUGIN_DESCRIPTION}")

if (PLUGIN_GSOAP)

    pkg_check_modules(GSOAP REQUIRED gsoap++)

    # Services

    # In order to clean up the mess in CLion, we kind of have to generated the sources outside of the build directory
    # It keeps it from complaining about out of project source files
    set(GSOAP_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated)
    add_custom_command(OUTPUT ${GSOAP_GEN_DIR}/webservices/soapC.cpp ${GSOAP_GEN_DIR}/webservices/soapcontrolProxy.cpp ${GSOAP_GEN_DIR}/webservices/soapcontrolService.cpp
        COMMAND ${CMAKE_COMMAND} -E make_directory ${GSOAP_GEN_DIR}/webservices
        COMMAND soapcpp2 -c++11 -i -d ${GSOAP_GEN_DIR}/webservices ${CMAKE_CURRENT_SOURCE_DIR}/ctrl-webservice-definition.hpp
        )

    add_custom_target(webservices DEPENDS ${GSOAP_GEN_DIR}/webservices/soapC.cpp)

    include_directories(
        ${GSOAP_INCLUDE_DIRS}
    )

    link_libraries(
        ${GSOAP_LIBRARIES}
    )

    # WARNINGS
    add_definitions(-Wno-error=extra)
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 6.0)
      add_definitions(-Wno-error=misleading-indentation)
    endif()
    
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
        generated/webservices/soapC.cpp
        generated/webservices/soapcontrolProxy.cpp
        )

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

    install(TARGETS soapserver LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
