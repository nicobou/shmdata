pkg_check_modules(PYTHON python3)

option(WITH_TOOLS "Tools made with pyquid" ${PYTHON_FOUND})
add_feature_info("Tools" WITH_TOOLS "Tools based on pyquid")

if (WITH_TOOLS)
  install(DIRECTORY . DESTINATION bin
    PATTERN "shmdata2jack"
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    GROUP_EXECUTE GROUP_READ
    WORLD_EXECUTE WORLD_READ)
endif ()
