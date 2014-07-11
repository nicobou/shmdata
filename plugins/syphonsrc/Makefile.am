LIBTOOL = glibtool --tag=CXX

pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

#all:
#	sh build_syphon.sh

#clean: clean-am
#	rm -rf Syphon-framework/build

plugins_LTLIBRARIES = \
    libsyphonsrc.la 

#libsyphonsrc info
libsyphonsrc_la_SOURCES = \
    syphonsrc.cpp \
	syphonreader.mm

libsyphonsrc_la_include_HEADERS = \
    syphonsrc.h \
    syphonreader.h

libsyphonsrc_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libsyphonsrc_la_LDFLAGS = \
	-framework Syphon \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libsyphonsrc_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) \
	-framework Syphon \
    -DG_LOG_DOMAIN=\"switcher\"

libsyphonsrc_la_OBJCXXFLAGS = \
	-framework Syphon


#### plugin unit test
AM_CFLAGS = \
    -I$(top_srcdir)

AM_CXXFLAGS =  \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

AM_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

check_PROGRAMS = \
    check_syphonsrc_plugin

check_syphonsrc_plugin_SOURCES = \
    check_syphonsrc_plugin.cpp

TESTS = $(check_PROGRAMS)
