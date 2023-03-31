include(CheckCXXCompilerFlag)

pkg_check_modules(PYTHON python3)

option(WITH_PYTHON "Python Wrapper" ${PYTHON_FOUND})
add_feature_info("wrapper-python" WITH_PYTHON "Python Wrapper")

if (WITH_PYTHON)

    find_package(PythonInterp 3 REQUIRED)
    pkg_check_modules(PYTHON REQUIRED python3)

    link_directories(${PYTHON_LIBRARY_DIRS})
    add_compile_options(${PYTHON_CFLAGS})
    add_definitions(-Wno-error=missing-field-initializers)
    check_cxx_compiler_flag(-Wno-error=cast-function-type CAN_CAST_FUNCTION_TYPE)
    if(CAN_CAST_FUNCTION_TYPE)
        add_definitions(-Wno-error=cast-function-type)
    endif()

    add_library(pyshmdata SHARED
        pyshmdata.cpp
        )
    set_target_properties(pyshmdata PROPERTIES PREFIX "")

    target_include_directories(pyshmdata
        PRIVATE ${SHMDATA_DIR}
        PRIVATE ${PYTHON_INCLUDE_DIRS}
    )

    target_link_libraries(pyshmdata
        PRIVATE ${SHMDATA_LIBRARY}
        PRIVATE ${PYTHON_LIBRARIES}
    )

    execute_process ( COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('DESTSHARED'))" OUTPUT_VARIABLE PYTHON_DIST_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

    # Install

    install(TARGETS pyshmdata
      LIBRARY DESTINATION ${PYTHON_DIST_PACKAGES}
      COMPONENT lib)

    install(TARGETS pyshmdata
      LIBRARY DESTINATION ${PYTHON_DIST_PACKAGES}
      COMPONENT dev)

    # Test
    add_test(pyshmdata_basic python3 ${CMAKE_CURRENT_SOURCE_DIR}/example.py)
    set_tests_properties(pyshmdata_basic PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}")
endif ()
