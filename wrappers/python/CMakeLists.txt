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
    execute_process ( COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" OUTPUT_VARIABLE PYTHON_DIST_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

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
    add_test(pyquid_quid_infotree_stress ${CMAKE_CURRENT_SOURCE_DIR}/examples/08-quid-infotree-stress.py)
    add_test(pyquid_uri_in_video_window ${CMAKE_CURRENT_SOURCE_DIR}/examples/09-uri-in-video-window.py)
    add_test(pyquid_quid_conf_and_two_jack_servers ${CMAKE_CURRENT_SOURCE_DIR}/examples/10-quid-conf-and-two-jack-servers.py)

endif ()
