link_libraries(
    ${SHMDATA_LIBRARY}
)

# SDFlow

option(WITH_SDFLOW "SDFlow Command Line" ON)
add_feature_info("sdflow" WITH_SDFLOW "SDFlow Command Line")
if (WITH_SDFLOW)

    add_executable(sdflow
        sdflow.cpp
        )

    # INSTALL

    install(TARGETS sdflow
        RUNTIME
        DESTINATION bin
        COMPONENT applications
        )

endif ()

# SDCrash

option(WITH_SDCRASH "SDCrash Command Line" ON)
add_feature_info("sdcrash" WITH_SDCRASH "SDCrash Command Line")
if (WITH_SDCRASH)

    add_executable(sdcrash
        sdcrash.cpp
        )

    # INSTALL

    install(TARGETS sdcrash
        RUNTIME
        DESTINATION bin
        COMPONENT applications
        )

endif ()
