# CMake Config
cmake_minimum_required(VERSION 3.5.1)

#
# CONFIGURATION
#

# Project Variables
set(SHMDATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(SHMDATA_VERSION_MAJOR 1)
set(SHMDATA_VERSION_MINOR 3)
set(SHMDATA_VERSION_PATCH 21)
set(SHMDATA_VERSION_STRING ${SHMDATA_VERSION_MAJOR}.${SHMDATA_VERSION_MINOR}.${SHMDATA_VERSION_PATCH})
set(SHMDATA_API_VERSION ${SHMDATA_VERSION_MAJOR}.${SHMDATA_VERSION_MINOR})
set(SHMDATA_LIBRARY shmdata-${SHMDATA_API_VERSION})

# Package (pkg-config) Information
set(PACKAGE_NAME shmdata)
set(PACKAGE_DESCRIPTION "Shares data flows between applications")
set(PACKAGE_URL "https://github.com/sat-metalab/shmdata")
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
set(CPACK_PACKAGE_VENDOR "Métalab - Société des arts technologiques")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PACKAGE_DESCRIPTION})
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR "${SHMDATA_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${SHMDATA_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${SHMDATA_VERSION_PATCH}")
set(CPACK_GENERATOR TGZ)
set(CPACK_SOURCE_GENERATOR TGZ)
set(CPACK_COMPONENTS_ALL applications libraries headers)

# CPack - Applications
set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME "SDFlow & SDCrash Applications")
set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION "Application to control test & debug shmdata")
set(CPACK_COMPONENT_APPLICATIONS_DEPENDS libraries)

# CPack - Libraries
set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION "Shared libraries used to build programs with shmdata")

# CPack - Includes
set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers")
set(CPACK_COMPONENT_HEADERS_DESCRIPTION "C/C++ header files for use with shmdata")
set(CPACK_COMPONENT_HEADERS_DEPENDS libraries)

# CPack - Source
set(CPACK_SOURCE_IGNORE_FILES "/\.hooks/;/\.idea/;/build/;/html/;${CPACK_SOURCE_IGNORE_FILES}")

# Add local cmake directory to ckame module path in case we need it
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
