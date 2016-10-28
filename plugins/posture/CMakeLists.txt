# PLUGIN

pkg_check_modules(POSTURE posture)

set(PLUGIN_NAME "plugin-posture")
set(PLUGIN_DESCRIPTION "Posture Plugin")

option(PLUGIN_POSTURE "${PLUGIN_DESCRIPTION}" ${POSTURE_FOUND})
add_feature_info("${PLUGIN_NAME}" PLUGIN_POSTURE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_POSTURE)

    pkg_check_modules(POSTURE REQUIRED posture)

    add_compile_options(${POSTURE_CFLAGS})

    include_directories(
        ${POSTURE_INCLUDE_DIRS}
    )

    link_libraries(
        ${POSTURE_LIBRARIES}
    )

    # Colorize

    add_library(posturecolorize SHARED
        posture_colorize.cpp
        posture_worker.cpp
        )

    add_dependencies(posturecolorize ${SWITCHER_LIBRARY})

    # Colorize GL

    add_library(posturecolorizeGL SHARED
        posture_colorizeGL.cpp
        posture_worker.cpp
        )

    add_dependencies(posturecolorizeGL ${SWITCHER_LIBRARY})

    # Detect

    add_library(posturedetect SHARED
        posture_detect.cpp
        )

    add_dependencies(posturedetect ${SWITCHER_LIBRARY})

    # Display

    add_library(posturedisplay SHARED
        posture_display.cpp
        )

    add_dependencies(posturedisplay ${SWITCHER_LIBRARY})

    # Merge

    add_library(posturemerge SHARED
        posture_merge.cpp
        )

    add_dependencies(posturemerge ${SWITCHER_LIBRARY})

    # Mesh Merge

    add_library(posturemeshmerge SHARED
        posture_meshmerge.cpp
        posture_worker.cpp
        )

    add_dependencies(posturemeshmerge ${SWITCHER_LIBRARY})

    # Mesh Serialization

    add_library(posturemeshserialization SHARED
        posture_meshserialization.cpp
        )

    add_dependencies(posturemeshserialization ${SWITCHER_LIBRARY})

    # Solidify

    add_library(posturesolidify SHARED
        posture_solidify.cpp
        posture_worker.cpp
        )

    add_dependencies(posturesolidify ${SWITCHER_LIBRARY})

    # Source

    add_library(posturesource SHARED
        posture_source.cpp
        )

    add_dependencies(posturesource ${SWITCHER_LIBRARY})

    # Scan 3D

    add_library(posturescan3d SHARED
        posture_scan3d.cpp
        )

    add_dependencies(posturescan3d ${SWITCHER_LIBRARY})

    # Scan 3D GPU

    add_library(posturescan3dGPU SHARED
        posture_scan3dGPU.cpp
        )

    add_dependencies(posturescan3dGPU ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_posture check_posture.cpp)
    add_test(check_posture check_posture)

    add_executable(check_rtp_posture check_rtp_posture.cpp)
    add_test(check_rtp_posture check_rtp_posture)

    # INSTALL

    install(TARGETS
        posturecolorize
        posturecolorizeGL
        posturedetect
        posturedisplay
        posturemerge
        posturemeshmerge
        posturemeshserialization
        posturesolidify
        posturesource
        posturescan3d
        posturescan3dGPU
        LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()