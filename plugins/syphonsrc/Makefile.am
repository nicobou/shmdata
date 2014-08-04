LIBTOOL = glibtool --tag=CXX

pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

clean: clean-am
	rm -rf Syphon-framework/build

install: install-am
	sh build_syphon.sh install
	sh build_syphon.sh clean

plugins_LTLIBRARIES = \
    libsyphonsrc.la 

#libsyphonsrc info
libsyphonsrc_la_SOURCES = \
    syphonsrc.cpp \
    syphonreader.mm

noinst_HEADERS = \
    syphonsrc.h \
    syphonreader.h

libsyphonsrc_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libsyphonsrc_la_LDFLAGS = \
    -F$(top_srcdir)/plugins/syphonsrc/Syphon-framework/build/Release \
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
    -F$(top_srcdir)/plugins/syphonsrc/Syphon-framework/build/Release \
    -framework Syphon \
    -DG_LOG_DOMAIN=\"switcher\"

libsyphonsrc_la_OBJCXXFLAGS = \
    -F$(top_srcdir)/plugins/syphonsrc/Syphon-framework/build/Release \
    -framework Syphon

# Needed to build Syphon.framework before syphonsrc
if HAVE_OSX
BUILT_SOURCES = \
    Syphon-framework/build/Release/Syphon.framework/Headers/Syphon.h

Syphon-framework/build/Release/Syphon.framework/Headers/Syphon.h: Makefile
	git submodule init && git submodule update && sh build_syphon.sh
endif
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
