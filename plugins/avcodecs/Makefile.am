pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libaac.la \
    libh264.la

#libaac info
libaac_la_SOURCES = \
    aac.cpp

libaac_la_include_HEADERS = \
    aac.h

libaac_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libaac_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libaac_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 


#libh264 info
libh264_la_SOURCES = \
    h264.cpp

libh264_la_include_HEADERS = \
    h264.h

libh264_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libh264_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libh264_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 
