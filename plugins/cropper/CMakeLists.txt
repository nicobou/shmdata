# PLUGIN

set(PLUGIN_NAME "plugin-cropper")
set(PLUGIN_DESCRIPTION "Video Cropper Plugin")

option(PLUGIN_CROPPER "${PLUGIN_DESCRIPTION}" TRUE)
add_feature_info("${PLUGIN_NAME}" PLUGIN_CROPPER "${PLUGIN_DESCRIPTION}")

if (PLUGIN_CROPPER)
    
    add_library(cropper SHARED
        cropper.cpp
        )

    add_dependencies(cropper ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_cropper check_cropper.cpp)
    add_test(check_cropper check_cropper)

    add_executable(check_cropper_caps check_cropper_caps.cpp)
    add_test(check_cropper_caps check_cropper_caps)

    # INSTALL

    install(TARGETS cropper LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
