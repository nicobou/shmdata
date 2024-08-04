# SDFlow

option(WITH_SDFLOW "SDFlow Command Line" ON)
add_feature_info("sdflow" WITH_SDFLOW "SDFlow Command Line")
if (WITH_SDFLOW)

    add_executable(sdflow
        sdflow.cpp
        )

    target_link_libraries(sdflow PUBLIC ${SHMDATA_LIBRARY})
      
    # INSTALL

    install(TARGETS sdflow
        RUNTIME
        DESTINATION bin
        COMPONENT lib
        )

    install(TARGETS sdflow
        RUNTIME
        DESTINATION bin
        COMPONENT dev
        )

    install(TARGETS sdflow
        RUNTIME
        DESTINATION bin
        COMPONENT dbg
        )

endif ()

# SDCrash

option(WITH_SDCRASH "SDCrash Command Line" ON)
add_feature_info("sdcrash" WITH_SDCRASH "SDCrash Command Line")
if (WITH_SDCRASH)

    add_executable(sdcrash
        sdcrash.cpp
        )

    target_link_libraries(sdcrash PUBLIC ${SHMDATA_LIBRARY})

    # INSTALL

    install(TARGETS sdcrash
        RUNTIME
        DESTINATION bin
        COMPONENT dev
        )

    install(TARGETS sdcrash
        RUNTIME
        DESTINATION bin
        COMPONENT dbg
        )

endif ()
