configure_file("cmake/PackageSourceTest.sh.in" "PackageSourceTest.sh" @ONLY)
add_custom_target(package_source_test COMMAND ${CMAKE_CURRENT_BINARY_DIR}/PackageSourceTest.sh)