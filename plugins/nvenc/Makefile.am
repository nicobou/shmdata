pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libnvenc-plugin.la 

#libnvenc-plugin info
libnvenc_plugin_la_SOURCES = \
    cuda/cuda-context.cpp \
    encoder/nvenc-api.cpp \
    encoder/nvenc-buffers.cpp \
    encoder/nvenc-encode-session.cpp \
    encoder/nvenc-plugin.cpp

libnvenc_plugin_la_include_HEADERS = \
    cuda/cu-res.hpp \
    cuda/cuda-context.hpp \
    encoder/nvenc-api.hpp \
    encoder/nvenc-buffers.hpp \
    encoder/nvenc-encode-session.hpp \
    encoder/nvenc-plugin.hpp

libnvenc_plugin_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libnvenc_plugin_la_LDFLAGS = \
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
    -lnvidia-encode \
    -lcuda

libnvenc_plugin_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(CUBA_LIBS) \
    $(NVENC_LIBS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) \
    -I$(NVENC_SDK_PATH)/Samples/common/inc/


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
    $(CUBA_LIBS) \
    $(NVENC_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

check_PROGRAMS = \
    check_nvenc

check_nvenc_SOURCES = \
    encoder/check_nvenc.cpp

TESTS = $(check_PROGRAMS)
