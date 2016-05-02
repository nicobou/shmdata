pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libposturecolorize.la \
    libposturecolorizeGL.la \
    libposturedetect.la \
    libposturedisplay.la \
    libposturemerge.la \
    libposturemeshmerge.la \
    libposturescan3d.la \
    libposturemeshserialization.la \
    libposturesource.la \
    libposturesolidify.la \
    libposturescan3dGPU.la

#libposturecolorize info
libposturecolorize_la_SOURCES = \
    posture_colorize.cpp \
    posture_worker.cpp

libposturecolorize_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturecolorize_la_include_HEADERS = \
    posture.hpp \
    posture_colorize.hpp \
    posture_worker.hpp

#libposturecolorizeGL info
libposturecolorizeGL_la_SOURCES = \
    posture_colorizeGL.cpp \
    posture_worker.cpp

libposturecolorizeGL_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libposturecolorizeGL_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturecolorizeGL_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

libposturecolorizeGL_la_include_HEADERS = \
    posture_colorizeGL.hpp \
    posture_worker.hpp

#libposturedetect info
libposturedetect_la_SOURCES = \
    posture_detect.cpp

libposturedetect_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturedetect_la_include_HEADERS = \
    posture_detect.hpp

#libposturedisplay info
libposturedisplay_la_SOURCES = \
    posture_display.cpp

libposturedisplay_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturedisplay_la_include_HEADERS = \
    posture_display.hpp

#libposturemerge info
libposturemerge_la_SOURCES = \
    posture_merge.cpp

libposturemerge_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturemerge_la_include_HEADERS = \
    posture_merge.hpp

#libposturemeshmerge info
libposturemeshmerge_la_SOURCES = \
    posture_meshmerge.cpp \
    posture_worker.cpp

libposturemeshmerge_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturemeshmerge_la_include_HEADERS = \
    posture_meshmerge.hpp \
    posture_worker.hpp

#libposturemeshserialization info
libposturemeshserialization_la_SOURCES = \
    posture_meshserialization.cpp

libposturemeshserialization_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libposturemeshserialization_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturemeshserialization_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

libposturemeshserialization_la_include_HEADERS = \
    posture_meshserialization.hpp

#libposturesolidify info
libposturesolidify_la_SOURCES = \
    posture_solidify.cpp \
    posture_worker.cpp

libposturesolidify_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturesolidify_la_include_HEADERS = \
    posture_solidify.hpp \
    posture_worker.hpp

#libposturesource info
libposturesource_la_SOURCES = \
    posture_source.cpp

libposturesource_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturesource_la_include_HEADERS = \
    posture_source.hpp

#libposturescan3d info
libposturescan3d_la_SOURCES = \
    posture_scan3d.cpp

libposturescan3d_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

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

libposturescan3d_la_include_HEADERS = \
    posture_scan3d.hpp

#libposturescan3dGPU info
libposturescan3dGPU_la_SOURCES = \
    posture_scan3dGPU.cpp

libposturescan3dGPU_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libposturescan3dGPU_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(POSTURE_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libposturescan3dGPU_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(POSTURE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"switcher\"

libposturescan3dGPU_la_include_HEADERS = \
    posture_scan3dGPU.hpp

#### plugin unit test
 AM_CFLAGS = \
     -I$(top_srcdir)

 AM_CXXFLAGS =  \
     $(GLIB_CFLAGS) \
     $(GSOAP_CFLAGS) \
     $(GST_CFLAGS) \
     $(JSONGLIB_CFLAGS) \
     $(POSTURE_CFLAGS) \
     $(SHMDATA_CFLAGS) \
     -I$(top_srcdir)

 AM_LDFLAGS = \
     $(GLIB_LIBS) \
     $(GSOAP_LIBS) \
     $(GST_LIBS) \
     $(JSONGLIB_LIBS) \
     $(POSTURE_LIBS) \
     $(SHMDATA_LIBS) \
     $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

 check_PROGRAMS = \
     check_posture \
     check_rtp_posture

 check_posture_SOURCES = \
     check_posture.cpp

 check_rtp_posture_SOURCES = \
     check_rtp_posture.cpp

TESTS = $(check_PROGRAMS)
