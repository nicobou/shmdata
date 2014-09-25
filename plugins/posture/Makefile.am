pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libposturedisplay.la \
    libposturemerge.la \
		libposturesolidify.la \
    libposturesource.la 

#libposturedisplay info
libposturedisplay_la_SOURCES = \
    posture_display.cpp

libposturedisplay_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturedisplay_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS = \
    posture_display.hpp

#libposturemerge info
libposturemerge_la_SOURCES = \
    posture_merge.cpp

libposturemerge_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturemerge_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS += \
    posture_merge.hpp

#libposturesolidify info
libposturesolidify_la_SOURCES = \
    posture_solidify.cpp

libposturesolidify_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturesolidify_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS += \
    posture_solidify.hpp

#libposturesource info
libposturesource_la_SOURCES = \
    posture_source.cpp

libposturesource_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturesource_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS += \
    posture.hpp \
    posture_source.hpp

#### plugin unit test
# AM_CFLAGS = \
#     -I$(top_srcdir)

# AM_CXXFLAGS =  \
#     $(GLIB_CFLAGS) \
#     $(GSOAP_CFLAGS) \
#     $(GST_CFLAGS) \
#     $(JSONGLIB_CFLAGS) \
# 	$(POSTURE_CFLAGS) \
#     $(SHMDATA_CFLAGS) \
#     -I$(top_srcdir) 

# AM_LDFLAGS = \
#     $(GLIB_LIBS) \
#     $(GSOAP_LIBS) \
#     $(GST_LIBS) \
#     $(JSONGLIB_LIBS) \
# 	$(POSTURE_LIBS) \
#     $(SHMDATA_LIBS) \
#     $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

# check_PROGRAMS = \
#     check_posture_plugin

# check_posture_plugin_SOURCES = \
#     check_posture_plugin.cpp

#TESTS = $(check_PROGRAMS)
