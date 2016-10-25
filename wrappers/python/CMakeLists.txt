find_package(PythonLibs 3)

option(WITH_PYTHON "Python Wrapper" ${PYTHONLIBS_FOUND})
add_feature_info("wrapper-python" WITH_PYTHON "Python Wrapper")

if (WITH_PYTHON AND PythonLibs_FOUND)

    find_package(PythonLibs 3 REQUIRED)

    add_compile_options(${PythonLibs_CFLAGS})
    add_definitions(-Wno-error=missing-field-initializers)

    add_library(pyshmdata SHARED
        pyshmdata.cpp
        )
    set_target_properties(pyshmdata PROPERTIES PREFIX "")

    target_include_directories(pyshmdata PRIVATE ${SHMDATA_DIR} ${PYTHON_INCLUDE_DIRS})
    target_link_libraries(pyshmdata PRIVATE ${SHMDATA_LIBRARY} ${PYTHON_LIBRARIES})

    execute_process(COMMAND python3 -c "import sys; from distutils import sysconfig; print((sysconfig.get_python_lib(1,0)).split(sys.prefix+'/')[1])"
        OUTPUT_VARIABLE PYTHON_DIST_PACKAGES
        )

    # The result of the preceding command end with a line return, we have to get rid of it
    string(REGEX REPLACE "(.*)[\n\r]" "\\1" PYTHON_DIST_PACKAGES ${PYTHON_DIST_PACKAGES} )

    # INSTALL

    install(TARGETS pyshmdata LIBRARY DESTINATION ${PYTHON_DIST_PACKAGES})

endif ()