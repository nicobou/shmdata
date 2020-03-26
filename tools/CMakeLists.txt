pkg_check_modules(PYTHON python3)

option(WITH_TOOLS "Tools made with pyquid" ${PYTHON_FOUND})
add_feature_info("Tools" WITH_TOOLS "Tools based on pyquid")

if (WITH_TOOLS)
  install(DIRECTORY . DESTINATION bin
    FILES_MATCHING PATTERN "s*"
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    GROUP_EXECUTE GROUP_READ
    WORLD_EXECUTE WORLD_READ)

  add_test(NAME check_swcam_display COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_swcam_display.sh)
  add_test(NAME check_shmshot COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_shmshot.sh)
  add_test(NAME check_shmdata2jack COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_shmdata2jack.sh)

endif ()
