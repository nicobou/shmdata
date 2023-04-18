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

  # PYQUID_SCRIPTS can be configured using -DPYQUID_SCRIPTS=/path/to/pyquid/scripts in the cmake command line
  set(PYQUID_SCRIPTS ${CMAKE_INSTALL_PREFIX}/${SWITCHER_LIBRARY}/pyquid)
  set(PYQUID_SCRIPTS_LOCAL ${CMAKE_CURRENT_SOURCE_DIR})

  # compile time definition to locate pyquid scripts for interpreted types
  add_compile_definitions(PYQUID_SCRIPTS_DIR="${PYQUID_SCRIPTS}")
  add_compile_definitions(PYQUID_SCRIPTS_DIR_LOCAL="${PYQUID_SCRIPTS_LOCAL}")


  add_library(pyquid SHARED
    pyfclaw.cpp
    pyinfotree.cpp
    pyquid.cpp
    pyquiddity.cpp
    pyquiddities.cpp
    pysession.cpp
    pyswitch.cpp
    pylogger.cpp
    pywclaw.cpp
    pybundle.cpp
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

  # Install library
  install(TARGETS pyquid LIBRARY DESTINATION ${DESTSHARED} COMPONENT lib)
  # Install library for the 'dev' cmake component
  install(TARGETS pyquid LIBRARY DESTINATION ${DESTSHARED} COMPONENT dev)
  # Install library for the 'dev' cmake component
  install(TARGETS pyquid LIBRARY DESTINATION ${DESTSHARED} COMPONENT dbg)

  # install python files
  install(FILES signals.py props.py bundles.py
    DESTINATION ${SWITCHER_LIBRARY}/pyquid
    COMPONENT lib)
  # install python files  for the 'dev' cmake component
  install(FILES signals.py props.py bundles.py
    DESTINATION ${SWITCHER_LIBRARY}/pyquid
    COMPONENT dev)
  # install python files  for the 'dev' cmake component
  install(FILES signals.py props.py bundles.py
    DESTINATION ${SWITCHER_LIBRARY}/pyquid
    COMPONENT dbg)

  # ADD EXAMPLE TESTS
  file(GLOB EXAMPLE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.py")
  list(FILTER EXAMPLE_FILES EXCLUDE REGEX "16-register-sip")

  foreach(EXAMPLE_FILE ${EXAMPLE_FILES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    add_test(pyquid_${EXAMPLE_NAME} ${EXAMPLE_FILE})
  endforeach()

endif ()
