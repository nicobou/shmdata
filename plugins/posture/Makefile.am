pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libposturescan3d.la \
	libposturecolorize.la \
	libposturedetect.la \
	libposturedisplay.la \
	libposturemerge.la \
	libposturemeshmerge.la \
	libposturesource.la \
	libposturesolidify.la

noinst_HEADERS = \
	posture.hpp

#libposturecolorize info
libposturecolorize_la_SOURCES = \
	posture_colorize.cpp \
	posture_worker.cpp

libposturecolorize_la_LDFLAGS = \
	$(GLIB_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_LIBS) \
	$(JSONGLIB_LIBS) \
	$(POSTURE_LIBS) \
	$(SHMDATA_LIBS) \
	-avoid-version \
	-L$(top_builddir)/switcher/.libs \
	-lswitcher-@LIBSWITCHER_API_VERSION@

libposturecolorize_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(GMODULE_CFLAGS) \
	$(GST_CFLAGS) \
	$(JSONGLIB_CFLAGS) \
	$(POSTURE_CFLAGS) \
	$(SHMDATA_CFLAGS) \
	-I$(top_srcdir) \
	-DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS += \
	posture_colorize.hpp \
	posture_worker.hpp

#libposturedetect info
libposturedetect_la_SOURCES = \
	posture_detect.cpp

libposturedetect_la_LDFLAGS = \
	$(GLIB_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_LIBS) \
	$(JSONGLIB_LIBS) \
	$(POSTURE_LIBS) \
	$(SHMDATA_LIBS) \
	-avoid-version \
	-L$(top_builddir)/switcher/.libs \
	-lswitcher-@LIBSWITCHER_API_VERSION@

libposturedetect_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(GMODULE_CFLAGS) \
	$(GST_CFLAGS) \
	$(JSONGLIB_CFLAGS) \
	$(POSTURE_CFLAGS) \
	$(SHMDATA_CFLAGS) \
	-I$(top_srcdir) \
	-DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS += \
	posture_detect.hpp

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

noinst_HEADERS += \
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

#libposturemeshmerge info
libposturemeshmerge_la_SOURCES = \
	posture_meshmerge.cpp \
	posture_worker.cpp

libposturemeshmerge_la_LDFLAGS = \
	$(GLIB_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_LIBS) \
	$(JSONGLIB_LIBS) \
	$(POSTURE_LIBS) \
	$(SHMDATA_LIBS) \
	-avoid-version \
	-L$(top_builddir)/switcher/.libs \
	-lswitcher-@LIBSWITCHER_API_VERSION@

libposturemeshmerge_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(GMODULE_CFLAGS) \
	$(GST_CFLAGS) \
	$(JSONGLIB_CFLAGS) \
	$(POSTURE_CFLAGS) \
	$(SHMDATA_CFLAGS) \
	-I$(top_srcdir) \
	-DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS += \
	posture_meshmerge.hpp \
	posture_worker.hpp

#libposturesolidify info
libposturesolidify_la_SOURCES = \
	posture_solidify.cpp \
	posture_worker.cpp

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
	posture_solidify.hpp \
	posture_worker.hpp

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
	posture_source.hpp

#libposturescan3d info
libposturescan3d_la_SOURCES = \
    posture_scan3d.cpp

libposturescan3d_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturescan3d_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

noinst_HEADERS = \
    posture_scan3d.hpp

#### plugin unit test
# AM_CFLAGS = \
#     -I$(top_srcdir)

# AM_CXXFLAGS =  \
#     $(GLIB_CFLAGS) \
#     $(GSOAP_CFLAGS) \
#     $(GST_CFLAGS) \
#     $(JSONGLIB_CFLAGS) \
#     $(POSTURE_CFLAGS) \
#     $(SHMDATA_CFLAGS) \
#     -I$(top_srcdir) 

# AM_LDFLAGS = \
#     $(GLIB_LIBS) \
#     $(GSOAP_LIBS) \
#     $(GST_LIBS) \
#     $(JSONGLIB_LIBS) \
#     $(POSTURE_LIBS) \
#     $(SHMDATA_LIBS) \
#     $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

# check_PROGRAMS = \
#     check_posture_plugin

# check_posture_plugin_SOURCES = \
#     check_posture_plugin.cpp

#TESTS = $(check_PROGRAMS)
