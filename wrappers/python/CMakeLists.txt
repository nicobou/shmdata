pkg_check_modules(PYTHON python3)

option(WITH_PYTHON "Python Wrapper" ${PYTHON_FOUND})
add_feature_info("wrapper-python" WITH_PYTHON "Python Wrapper")

if (WITH_PYTHON)

  find_package(PythonInterp 3 REQUIRED)
  pkg_check_modules(PYTHON REQUIRED python3)

    link_directories(${PYTHON_LIBRARY_DIRS})
    add_compile_options(${PYTHON_CFLAGS})
    add_definitions(-Wno-error=missing-field-initializers)

    add_library(pyquid SHARED
        pyinfotree.cpp
        pyquid.cpp
	pyquiddity.cpp
	pyqrox.cpp
	pyswitch.cpp
        )
    set_target_properties(pyquid PROPERTIES PREFIX "")

    target_include_directories(pyquid
        PRIVATE ${SWITCHER_DIR}
        PRIVATE ${PYTHON_INCLUDE_DIRS}
    )

    target_link_libraries(pyquid
        PRIVATE ${SWITCHER_LIBRARY}
        PRIVATE ${PYTHON_LIBRARIES}
    )

    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sys; from distutils import sysconfig; print((sysconfig.get_python_lib(1,0)).split(sys.prefix+'/')[1])"
        OUTPUT_VARIABLE PYTHON_DIST_PACKAGES
        )

    # The result of the preceding command end with a line return, we have to get rid of it
    string(REGEX REPLACE "(.*)[\n\r]" "\\1" PYTHON_DIST_PACKAGES ${PYTHON_DIST_PACKAGES} )

    # INSTALL

    install(TARGETS pyquid LIBRARY DESTINATION ${PYTHON_DIST_PACKAGES})

    # TEST
    add_test(pyquid_basic ${CMAKE_CURRENT_SOURCE_DIR}/examples/01-basic.py)
    add_test(pyquid_introspection ${CMAKE_CURRENT_SOURCE_DIR}/examples/02-introspection.py)
    add_test(pyquid_infotree ${CMAKE_CURRENT_SOURCE_DIR}/examples/03-infotree.py)
    add_test(pyquid_signal ${CMAKE_CURRENT_SOURCE_DIR}/examples/04-signal.py)
    add_test(pyquid_save ${CMAKE_CURRENT_SOURCE_DIR}/examples/05-save.py)
    add_test(pyquid_property_sub ${CMAKE_CURRENT_SOURCE_DIR}/examples/06-subscribe-to-property.py)
    add_test(pyquid_invoke_async ${CMAKE_CURRENT_SOURCE_DIR}/examples/07-invoke-async.py)

endif ()
