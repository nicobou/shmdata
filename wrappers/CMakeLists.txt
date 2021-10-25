link_libraries(
    ${SWITCHER_LIBRARY}
)

include_directories(
    ${SWITCHER_DIR}
)

add_subdirectory(python)
add_subdirectory(switcherio)

