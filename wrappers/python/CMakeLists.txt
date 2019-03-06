pkg_check_modules(PYTHON python3)

option(WITH_PYTHON "Python Wrapper" ${PYTHON_FOUND})
add_feature_info("wrapper-python" WITH_PYTHON "Python Wrapper")

if (WITH_PYTHON)

    find_package(PythonInterp 3 REQUIRED)
    pkg_check_modules(PYTHON REQUIRED python3)

    link_directories(${PYTHON_LIBRARY_DIRS})
    add_compile_options(${PYTHON_CFLAGS})
    add_definitions(-Wno-error=missing-field-initializers)

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

    execute_process ( COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" OUTPUT_VARIABLE PYTHON_DIST_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

    # Install

    install(TARGETS pyshmdata LIBRARY DESTINATION ${PYTHON_DIST_PACKAGES})

    # Test
    add_test(pyshmdata_basic ${CMAKE_CURRENT_SOURCE_DIR}/example.py)
endif ()
