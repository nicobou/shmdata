AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(PULSE_CFLAGS) \
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
    libpulsesink.la \
    libpulsesrc.la

libpulsesrc_la_SOURCES = \
    pulsesrc.cpp

libpulsesrc_la_include_HEADERS = \
    pulsesrc.h

libpulsesrc_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libpulsesrc_la_LDFLAGS = \
    $(PULSE_LIBS) 


libpulsesink_la_SOURCES = \
    pulsesink.cpp

libpulsesink_la_include_HEADERS = \
    pulsesink.h

libpulsesink_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libpulsesink_la_LDFLAGS = \
    $(PULSE_LIBS) 
