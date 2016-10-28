# PLUGIN

pkg_check_modules(GTK gtk+-3.0)

if(GTK_FOUND AND NOT OSX)
    set(ENABLED TRUE)
endif()

set(PLUGIN_NAME "plugin-gtk")
set(PLUGIN_DESCRIPTION "GTK Plugin")

option(PLUGIN_GTK "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_GTK "${PLUGIN_DESCRIPTION}")

if (PLUGIN_GTK)

    pkg_check_modules(GTK REQUIRED gtk+-3.0)

    add_compile_options(${GTK_CFLAGS})

    include_directories(
        ${GTK_INCLUDE_DIRS}
    )

    link_libraries(
        ${GTK_LIBRARIES}
    )

    add_library(gtkvideo SHARED
        gtkvideo.cpp
        )

    add_dependencies(gtkvideo ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_gtk_video check_gtk_video.cpp)
    add_test(check_gtk_video check_gtk_video)

    # INSTALL

    install(TARGETS gtkvideo LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
