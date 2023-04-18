# PLUGIN

set(PLUGIN_NAME "plugin-resample")
set(PLUGIN_DESCRIPTION "Libsamplerate")

cmake_dependent_option(PLUGIN_RESAMPLE "${PLUGIN_DESCRIPTION}" ${LIBSAMPLERATE_FOUND}
                       "ENABLE_GPL" OFF)
add_feature_info("${PLUGIN_NAME}" PLUGIN_RESAMPLE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_RESAMPLE)

    pkg_check_modules(LIBSAMPLERATE REQUIRED samplerate)

    add_compile_options(${LIBSAMPLERATE_CFLAGS})

    include_directories(
        ${LIBSAMPLERATE_INCLUDE_DIRS}
    )

    link_libraries(
	${LIBSAMPLERATE_LIBRARIES}
    )


  # Resample Quid

    add_library(resample SHARED
            resample.cpp
        )
    add_dependencies(resample ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_resample check_resample.cpp)
    configure_file(check_resample.json check_resample.json COPYONLY)
    add_test(check_resample check_resample)

    # INSTALL

    install(TARGETS resample LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
