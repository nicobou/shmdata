pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libosc-ctrl-server.la \
    libosc-to-property.la

AM_LDFLAGS = \
    $(LIBLO_LIBS) \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

AM_CXXFLAGS = \
    $(LIBLO_CFLAGS) \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 


#libosc_ctrl_server (control from osc)
libosc_ctrl_server_la_SOURCES = \
    osc-ctrl-server.cpp

libosc_ctrl_server_la_include_HEADERS = \
    osc-ctrl-server.h

libosc_ctrl_server_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

#libosc-to-property
libosc_to_property_la_SOURCES = \
    osc-to-property.cpp

libosc_to_property_la_include_HEADERS = \
    osc-to-property.h

libosc_to_property_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

#### plugin unit test
check_PROGRAMS = \
    check_osc

check_osc_SOURCES = \
    check_osc.cpp

TESTS = $(check_PROGRAMS)
