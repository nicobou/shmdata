# PLUGIN

set(NVIDIA_PATH /usr/lib/nvidia-* CACHE STRING "Location of the nvidia libraries")

file(GLOB NVIDIA_SUFFIXES RELATIVE /usr/lib ${NVIDIA_PATH})
list(SORT NVIDIA_SUFFIXES)
list(REVERSE NVIDIA_SUFFIXES)

find_library(CUDA cuda)
find_library(NVENC nvidia-encode PATH_SUFFIXES ${NVIDIA_SUFFIXES})
find_library(NVCUVID nvcuvid PATH_SUFFIXES ${NVIDIA_SUFFIXES})

set(ENABLED FALSE)
if (CUDA AND NVENC AND NVCUVID)
    set(ENABLED TRUE)
endif ()

set(PLUGIN_NAME "plugin-nvenc")
set(PLUGIN_DESCRIPTION "NVEnc Plugin")

option(PLUGIN_NVENC "${PLUGIN_DESCRIPTION}" ${ENABLED})
add_feature_info("${PLUGIN_NAME}" PLUGIN_NVENC "${PLUGIN_DESCRIPTION}")

if (PLUGIN_NVENC)

    if (NOT CUDA)
        message(FATAL_ERROR "cuda not found")
    endif ()

    if (NOT NVENC)
        message(FATAL_ERROR "nvidia-encode not found")
    endif ()

    if (NOT NVCUVID)
        message(FATAL_ERROR "nvcuvid not found")
    endif ()

    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR} # For "nvidia/*" and "cuda/*"
    )

    link_libraries(
        ${CUDA}
        ${NVENC}
        ${NVCUVID}
    )

    # Encoder

    add_library(nvenc_encoder_plugin SHARED
        cuda/cuda-context.cpp
        encoder/nvenc-api.cpp
        encoder/nvenc-buffers.cpp
        encoder/nvenc-encode-session.cpp
        encoder/nvenc-plugin.cpp
        )

    add_dependencies(nvenc_encoder_plugin ${SWITCHER_LIBRARY})

    # Decoder

    add_library(nvenc_decoder_plugin SHARED
        cuda/cuda-context.cpp
        decoder/nvdec-plugin.cpp
        decoder/nvdec-decode-session.cpp
        )

    add_dependencies(nvenc_decoder_plugin ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_nvenc encoder/check_nvenc.cpp)
    configure_file(encoder/check_decode.json check_decode.json COPYONLY)
    add_test(check_nvenc check_nvenc)

    # INSTALL

    install(TARGETS nvenc_encoder_plugin nvenc_decoder_plugin LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
