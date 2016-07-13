pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libnvenc-encoder-plugin.la \
    libnvenc-decoder-plugin.la 

#libnvenc-plugin info
libnvenc_encoder_plugin_la_SOURCES = \
    cuda/cuda-context.cpp \
    encoder/nvenc-api.cpp \
    encoder/nvenc-buffers.cpp \
    encoder/nvenc-encode-session.cpp \
    encoder/nvenc-plugin.cpp

libnvenc_encoder_plugin_la_include_HEADERS = \
    cuda/cu-res.hpp \
    cuda/cuda-context.hpp \
    encoder/nvenc-api.hpp \
    encoder/nvenc-buffers.hpp \
    encoder/nvenc-encode-session.hpp \
    encoder/nvenc-plugin.hpp

libnvenc_decoder_plugin_la_SOURCES = \
    cuda/cuda-context.cpp \
    decoder/nvdec-plugin.cpp \
    decoder/nvdec-decode-session.cpp 

libnvenc_decoder_plugin_la_include_HEADERS = \
    cuda/cu-res.hpp \
    cuda/cuda-context.hpp \
    decoder/nvdec-plugin.hpp \
    decoder/nvdec-decode-session.hpp 

libnvenc_encoder_plugin_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libnvenc_decoder_plugin_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

COMMON_LD_FLAGS = \ 
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(CUDA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    -lcuda

COMMON_CXX_FLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(NVENC_LIBS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir)

libnvenc_encoder_plugin_la_LDFLAGS  = \
    $(COMMON_LD_FLAGS) \
    -lnvidia-encode 
libnvenc_encoder_plugin_la_CXXFLAGS = \
    $(COMMON_CXX_FLAGS)
    

libnvenc_decoder_plugin_la_LDFLAGS = \
    $(COMMON_LD_FLAGS) \
    -lnvcuvid
libnvenc_decoder_plugin_la_CXXFLAGS = \
    $(COMMON_CXX_FLAGS)

#### plugin unit test
AM_CFLAGS = \
    -I$(top_srcdir)

AM_CXXFLAGS =  \
    $(GLIB_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

AM_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(NVENC_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

check_PROGRAMS = \
    check_nvenc

check_nvenc_SOURCES = \
    encoder/check_nvenc.cpp

TESTS = $(check_PROGRAMS)
