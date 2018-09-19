link_libraries(
    ${SWITCHER_LIBRARY}
)

add_executable(check_bundle check_bundle.cpp)
configure_file(check_bundle.config check_bundle.config COPYONLY)
add_test(check_bundle check_bundle)

add_executable(check_file_decoder check_file_decoder.cpp)
configure_file(oie.mp3 oie.mp3 COPYONLY)
add_test(check_file_decoder check_file_decoder)

add_executable(check_gst_pipeline check_gst_pipeline.cpp)
add_test(check_gst_pipeline check_gst_pipeline)

add_executable(check_information_tree check_information_tree.cpp)
add_test(check_information_tree check_information_tree)

add_executable(check_manager check_manager.cpp)
add_test(check_manager check_manager)

add_executable(check_quiddity_create_remove check_quiddity_create_remove.cpp)
add_test(check_quiddity_create_remove check_quiddity_create_remove)

add_executable(check_scope_guard check_scope_guard.cpp)
add_test(check_scope_guard check_scope_guard)

add_executable(check_shmdelay check_shmdelay.cpp)
add_test(check_shmdelay check_shmdelay)

add_executable(check_startable check_startable.cpp)
add_test(check_startable check_startable)

add_executable(check_test_full check_test_full.cpp)
add_test(check_test_full check_test_full)

add_executable(check_threaded_wrapper check_threaded_wrapper.cpp)
add_test(check_threaded_wrapper check_threaded_wrapper)

add_executable(check_ugstelem check_ugstelem.cpp)
add_test(check_ugstelem check_ugstelem)

add_executable(check_string_utils check_string_utils.cpp)
add_test(check_string_utils check_string_utils)
