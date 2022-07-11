link_libraries(
    ${SHMDATA_LIBRARY}
)

add_executable(check-c-wrapper check-c-wrapper.cpp)
add_test(check-c-wrapper check-c-wrapper)

add_executable(check-file-monitor check-file-monitor.cpp)
add_test(check-file-monitor check-file-monitor)

add_executable(check-follower check-follower.cpp)
add_test(check-follower check-follower)

add_executable(check-shmdata check-shmdata.cpp)
add_test(check-shmdata check-shmdata)

add_executable(check-shmdata-stress check-shmdata-stress.cpp)
add_test(check-shmdata-stress check-shmdata-stress)

add_executable(check-shm-resize check-shm-resize.cpp)
add_test(check-shm-resize check-shm-resize)

add_executable(check-shm-size check-shm-size.cpp)
add_test(check-shm-size check-shm-size)

add_executable(check-sysv-sem check-sysv-sem.cpp)
add_test(check-sysv-sem check-sysv-sem)

add_executable(check-sysv-shm check-sysv-shm.cpp)
add_test(check-sysv-shm check-sysv-shm)

add_executable(check-type-parser check-type-parser.cpp)
add_test(check-type-parser check-type-parser)

add_executable(check-unix-perms check-unix-perms.cpp)
add_test(check-unix-perms check-unix-perms)

add_executable(check-unix-socket check-unix-socket.cpp)
add_test(check-unix-socket check-unix-socket)

add_executable(check-writer-follower check-writer-follower.cpp)
add_test(check-writer-follower check-writer-follower)

add_executable(check-latency check-latency.cpp)
add_test(check-latency check-latency)

add_test(NAME check-sdflow COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check-sdflow.sh)
set_tests_properties(check-sdflow PROPERTIES ENVIRONMENT "PATH=${CMAKE_BINARY_DIR}/utils:${PATH}")

add_test(NAME check-sdcrash COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check-sdcrash.sh)
set_tests_properties(check-sdcrash PROPERTIES ENVIRONMENT "PATH=${CMAKE_BINARY_DIR}/utils:${PATH}")
