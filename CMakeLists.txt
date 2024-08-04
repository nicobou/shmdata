# CMake Config
cmake_minimum_required(VERSION 3.5.1)
project(shmdata)

#
# CONFIGURATION
#

# Project Variables
set(SHMDATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(SHMDATA_VERSION_MAJOR 1)
set(SHMDATA_VERSION_MINOR 3)
set(SHMDATA_VERSION_PATCH 77)
set(SHMDATA_VERSION_STRING ${SHMDATA_VERSION_MAJOR}.${SHMDATA_VERSION_MINOR}.${SHMDATA_VERSION_PATCH})
set(SHMDATA_API_VERSION ${SHMDATA_VERSION_MAJOR}.${SHMDATA_VERSION_MINOR})
set(SHMDATA_LIBRARY shmdata-${SHMDATA_API_VERSION})

# Package (pkg-config) Information
set(PACKAGE_NAME shmdata)
set(PACKAGE_DESCRIPTION "Shares data flows between applications")
set(PACKAGE_URL "https://github.com/nicobou/shmdata")
set(PACKAGE_VERSION ${SHMDATA_VERSION_STRING})

# Compilation
set(CMAKE_CXX_STANDARD 17)


#
# Code coverage
#

set(TEST_COVERAGE OFF CACHE BOOL "Enable test coverage")
if (TEST_COVERAGE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_custom_command(OUTPUT test_coverage
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/coverage
    COMMAND lcov --no-external --capture --initial
                 --directory ${CMAKE_SOURCE_DIR}
                 --exclude '${CMAKE_SOURCE_DIR}/tests/*'
                 --exclude '${CMAKE_SOURCE_DIR}/build/*'
                 --output-file coverage/shmdata_base.info
    COMMAND make test
    COMMAND lcov --no-external --capture
                 --directory ${CMAKE_SOURCE_DIR}
                 --exclude '${CMAKE_SOURCE_DIR}/tests/*'
                 --exclude '${CMAKE_SOURCE_DIR}/build/*'
                 --output-file coverage/shmdata.info
    COMMAND lcov --add-tracefile ${CMAKE_BINARY_DIR}/coverage/shmdata_base.info
                 --add-tracefile ${CMAKE_BINARY_DIR}/coverage/shmdata.info
                 --output-file ${CMAKE_BINARY_DIR}/coverage/shmdata_total.info
    COMMAND genhtml --output-directory ${CMAKE_BINARY_DIR}/coverage ${CMAKE_BINARY_DIR}/coverage/shmdata_total.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_custom_target(check_coverage DEPENDS test_coverage)
endif()  # TEST_COVERAGE 

# CPack - General
set(CPACK_PACKAGE_NAME ${PACKAGE_NAME})
set(CPACK_PACKAGE_VENDOR "Nicolas Bouillot")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PACKAGE_DESCRIPTION})
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.md")
set(CPACK_PACKAGE_VERSION_MAJOR "${SHMDATA_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${SHMDATA_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${SHMDATA_VERSION_PATCH}")
set(CPACK_GENERATOR DEB)
set(CPACK_SOURCE_GENERATOR TGZ)
set(CPACK_COMPONENTS_ALL lib dev dbg)

# CPack - Debian Package
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
set(CPACK_DEBIAN_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}")
set(CPACK_DEBIAN_PROJECT_HOMEPAGE "https://gitlab.com/nicobou/shmdata/")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_VENDOR} <nbou7110t@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Library to share streams of framed data between processes via shared memory. shmdata is server less: it requires applications to link data streams using socket path (e.g. \"/tmp/my_shmdata_stream\"). Shmdata is very fast and allows processes to access data streams without the need of extra copy. The communication paradigm is 1 to many, i.e., one writer is making available data frames to several followers. Followers and writers can hot connect & disconnect.")
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.md ${CMAKE_CURRENT_SOURCE_DIR}/NEWS.md
  COMPONENT lib
  DESTINATION share/doc/${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.md ${CMAKE_CURRENT_SOURCE_DIR}/NEWS.md
  COMPONENT dev
  DESTINATION share/doc/${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}/)

# CPack - Libraries
set(CPACK_COMPONENT_LIB_DISPLAY_NAME "Shmdata libraries")
set(CPACK_COMPONENT_LIB_DESCRIPTION "Shared libraries, monitoring tool (sdflow) and GStreamer elements")

# CPack - Development
set(CPACK_COMPONENT_DEV_DISPLAY_NAME "Shmdata development files and documentation")
set(CPACK_COMPONENT_DEV_DESCRIPTION "C++ Headers, utils and libraries")
set(CPACK_DEBIAN_DEV_PACKAGE_REPLACES lib)

# CPack - Debug
set(CPACK_COMPONENT_DBG_DISPLAY_NAME "Shmdata development files and documentation")
set(CPACK_COMPONENT_DBG_DESCRIPTION "C++ Headers, utils and libraries with debug symbols")
set(CPACK_DEBIAN_DBG_PACKAGE_REPLACES lib dev)

# CPack - Source
set(CPACK_SOURCE_IGNORE_FILES "/\.hooks/;/build/;/html/;${CPACK_SOURCE_IGNORE_FILES}")

# Add local cmake directory to cmake module path in case we need it
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#
# INCLUDES
#

include(FeatureSummary)
include(CPack)
include(PackageSourceTest)

#
# PROJECT
#

project(Shmdata)
enable_testing()

# Prerequisites
find_package(PkgConfig REQUIRED)

# Global Stuff

link_libraries(
    pthread
)

include_directories(
    ${SHMDATA_DIR} # Shmdata needs to include from itself using "shmdata/*"
)

# BUILD TYPE
if(CMAKE_BUILD_TYPE EQUAL "Debug")
    add_definitions(-DDEBUG)
endif()

# WARNINGS
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Werror)
add_definitions(-Wno-error=unused-parameter)

# OSX
if(APPLE AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    add_definitions(-DOSX)
endif ()

#
# MODULES
#

add_subdirectory(shmdata)
add_subdirectory(utils)
add_subdirectory(tests)
add_subdirectory(wrappers)
add_subdirectory(gst)

#
# OTHER TARGETS
#

# Documentation
find_package(Doxygen)
if (DOXYGEN_FOUND)
    configure_file("doxyfile.in" "${CMAKE_SOURCE_DIR}/doxyfile")
    add_custom_target(doc COMMAND ${DOXYGEN_EXECUTABLE} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/html/
        DESTINATION  share/doc/${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}/html/
        COMPONENT dev
        )
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/html/
        DESTINATION  share/doc/${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}/html/
        COMPONENT dbg
        )
endif ()

# Uninstall
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/CMakeUninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/CMakeUninstall.cmake)

#
# POST
#

# Report
feature_summary(WHAT ALL)
