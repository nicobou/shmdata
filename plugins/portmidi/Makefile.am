pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

lib_LTLIBRARIES = \
    libportmididev.la

#libportmididev info
libportmididev_la_SOURCES = \
    portmidi-devices.cpp

libportmididev_la_include_HEADERS = \
    portmidi-devices.h

libportmididev_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libportmididev_la_LDFLAGS = \
    $(PORTMIDI_LIBS) \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(SHMDATA_ANY_LIBS) \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libportmididev_la_CXXFLAGS = \
    $(PORTMIDI_CFLAGS) \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(SHMDATA_ANY_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

plugins_LTLIBRARIES = \
    libportmidisrc.la \
    libportmidisink.la

#libportmidisrc info
libportmidisrc_la_SOURCES = \
    portmidi-source.cpp

libportmidisrc_la_include_HEADERS = \
    portmidi-source.h

libportmidisrc_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libportmidisrc_la_LDFLAGS = \
    $(PORTMIDI_LIBS) \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(SHMDATA_ANY_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    $(top_builddir)/plugins/portmidi/libportmididev.la

libportmidisrc_la_CXXFLAGS = \
    $(PORTMIDI_CFLAGS) \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(SHMDATA_ANY_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

#libportmidisink info
libportmidisink_la_SOURCES = \
    portmidi-sink.cpp

libportmidisink_la_include_HEADERS = \
    portmidi-sink.h

libportmidisink_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libportmidisink_la_LDFLAGS = \
    $(PORTMIDI_LIBS) \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(SHMDATA_ANY_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    $(top_builddir)/plugins/portmidi/libportmididev.la

libportmidisink_la_CXXFLAGS = \
    $(PORTMIDI_CFLAGS) \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(SHMDATA_ANY_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

