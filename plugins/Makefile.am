AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

AM_LIBS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_CFLAGS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libmyplugin.la 

#libmyplugin info
libmyplugin_la_SOURCES = \
    myplugin.cpp

libmyplugin_la_include_HEADERS = \
    myplugin.h

libmyplugin_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

if V4L2BUILD
plugins_LTLIBRARIES += \
    libv4l2src.la
libv4l2src_la_SOURCES = \
    v4l2src.cpp

libv4l2src_la_include_HEADERS = \
    v4l2src.h

libv4l2src_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher
endif
