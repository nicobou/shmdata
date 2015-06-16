pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
	libvncplugin.la

libvncplugin_la_SOURCES = \
	vnc_client.cpp

libvncplugin_la_LDFLAGS = \
	$(GLIB_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_LIBS) \
	$(JSONGLIB_LIBS) \
	$(SHMDATA_LIBS) \
	$(VNC_LIBS) \
	-avoid-version \
	-L$(top_builddir)/switcher/.libs \
	-lswitcher-@LIBSWITCHER_API_VERSION@

libvncplugin_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(GMODULE_CFLAGS) \
	$(GST_CFLAGS) \
	$(JSONGLIB_CFLAGS) \
	$(SHMDATA_CFLAGS) \
	$(VNC_CFLAGS) \
	-I$(top_srcdir) \
	-DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS = \
	vnc_client.hpp
