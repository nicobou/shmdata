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
    libaac.la 

#libaac info
libaac_la_SOURCES = \
    aac.cpp

libaac_la_include_HEADERS = \
    aac.h

libaac_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

