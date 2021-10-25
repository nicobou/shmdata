include(CheckCXXCompilerFlag)
pkg_check_modules(PYTHON python3)

option(WITH_PYTHON "Python Wrapper" ${PYTHON_FOUND})
add_feature_info("wrapper-python" WITH_PYTHON "Python Wrapper")

if (WITH_PYTHON)
  find_package(PythonInterp 3 REQUIRED)
  pkg_check_modules(PYTHON REQUIRED python3)
  
  link_directories(${PYTHON_LIBRARY_DIRS})
  add_compile_options(${PYTHON_CFLAGS})
  add_compile_options("-Wno-missing-field-initializers")

  add_library(pyquid SHARED
    pyfclaw.cpp
    pyinfotree.cpp
    pyquid.cpp
    pyquiddity.cpp
    pysession.cpp
    pyswitch.cpp
    pywclaw.cpp
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

  # COMPUTE INSTALL PATH FOR PYQUID
  execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('DESTSHARED'))" OUTPUT_VARIABLE DESTSHARED OUTPUT_STRIP_TRAILING_WHITESPACE) 

  # INSTALL
  install(TARGETS pyquid LIBRARY DESTINATION ${DESTSHARED})

  # TESTS
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
  add_test(pyquid_switcher_signals ${CMAKE_CURRENT_SOURCE_DIR}/examples/11-switcher-signals.py)
  add_test(pyquid_switcher_session ${CMAKE_CURRENT_SOURCE_DIR}/examples/12-switcher-session.py)
  add_test(pyquid_shmdata_connection ${CMAKE_CURRENT_SOURCE_DIR}/examples/13-shmdata-connection.py)
  add_test(pyquid_shmdata_dynamic_claws ${CMAKE_CURRENT_SOURCE_DIR}/examples/14-shmdata-dynamic-claws.py)
endif ()

