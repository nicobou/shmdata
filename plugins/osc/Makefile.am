pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libosc-ctrl-server.la 

#libosc_ctrl_server (control from osc)
libosc_ctrl_server_la_SOURCES = \
    osc-ctrl-server.cpp

libosc_ctrl_server_la_include_HEADERS = \
    osc-ctrl-server.h

libosc_ctrl_server_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libosc_ctrl_server_la_LDFLAGS = \
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

libosc_ctrl_server_la_CXXFLAGS = \
    $(LIBLO_CFLAGS) \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 


