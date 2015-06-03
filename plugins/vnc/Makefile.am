pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
	libvncclient.la

libvncclient_la_SOURCES = \
	vnc_client.cpp

libvncclient_la_LDFLAGS = \
	$(GLIB_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_LIBS) \
	$(JSONGLIB_LIBS) \
	$(SHMDATA_LIBS) \
	$(VNC_LIBS) \
	-avoid-version \
	-L$(top_builddir)/switcher/.libs \
	-lswitcher-@LIBSWITCHER_API_VERSION@

libvncclient_la_CXXFLAGS = \
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
