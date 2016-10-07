pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libvrpnsrc.la \
    libvrpnsink.la

BUILT_SOURCES = \
    vrpn-lib/vrpn_Connection.h

LIBVRPN_INCLUDE_DIR = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

LIBVRPN_LD_FLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    -I./vrpn-lib/

LIBVRPN_CXX_FLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -Wno-error=unused-variable \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) \
    -I./vrpn-lib/

vrpn-lib/vrpn_Connection.h: Makefile
	cd vrpn-lib \
        && mkdir -p build \
        && cd build \
        && cmake .. \
        && make

clean-local:
	-rm -rf vrpn-lib/build

distclean-local:
	-rm -rf vrpn-lib/build

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
    check_vrpn_plugin

check_vrpn_plugin_SOURCES = \
    check_vrpn_plugin.cpp

TESTS = $(check_PROGRAMS)

#  ██╗   ██╗██████╗ ██████╗ ███╗   ██╗      ███████╗██████╗  ██████╗
#  ██║   ██║██╔══██╗██╔══██╗████╗  ██║      ██╔════╝██╔══██╗██╔════╝
#  ██║   ██║██████╔╝██████╔╝██╔██╗ ██║█████╗███████╗██████╔╝██║
#  ╚██╗ ██╔╝██╔══██╗██╔═══╝ ██║╚██╗██║╚════╝╚════██║██╔══██╗██║
#   ╚████╔╝ ██║  ██║██║     ██║ ╚████║      ███████║██║  ██║╚██████╗
#    ╚═══╝  ╚═╝  ╚═╝╚═╝     ╚═╝  ╚═══╝      ╚══════╝╚═╝  ╚═╝ ╚═════╝

libvrpnsrc_la_SOURCES = \
    $(LIBVRPN_SOURCES) \
    vrpn-source.cpp

nodist_libvrpnsrc_la_SOURCES = \
    vrpn-lib/build/libvrpn.a

libvrpnsrc_la_include_HEADERS = \
    vrpn-source.hpp

libvrpnsrc_la_includedir = \
    $(LIBVRPN_INCLUDE_DIR)

libvrpnsrc_la_LDFLAGS = \
    $(LIBVRPN_LD_FLAGS)

libvrpnsrc_la_CXXFLAGS = \
    $(LIBVRPN_CXX_FLAGS)

libvrpnsrc_la_LIBADD = \
    vrpn-lib/build/libvrpn.a

#  ██╗   ██╗██████╗ ██████╗ ███╗   ██╗      ███████╗██╗███╗   ██╗██╗  ██╗
#  ██║   ██║██╔══██╗██╔══██╗████╗  ██║      ██╔════╝██║████╗  ██║██║ ██╔╝
#  ██║   ██║██████╔╝██████╔╝██╔██╗ ██║█████╗███████╗██║██╔██╗ ██║█████╔╝
#  ╚██╗ ██╔╝██╔══██╗██╔═══╝ ██║╚██╗██║╚════╝╚════██║██║██║╚██╗██║██╔═██╗
#   ╚████╔╝ ██║  ██║██║     ██║ ╚████║      ███████║██║██║ ╚████║██║  ██╗
#    ╚═══╝  ╚═╝  ╚═╝╚═╝     ╚═╝  ╚═══╝      ╚══════╝╚═╝╚═╝  ╚═══╝╚═╝  ╚═╝

libvrpnsink_la_SOURCES = \
    vrpn-sink.cpp

nodist_libvrpnsink_la_SOURCES = \
    vrpn-lib/build/libvrpnserver.a

libvrpnsink_la_include_HEADERS = \
    vrpn-sink.hpp

libvrpnsink_la_includedir = \
    $(LIBVRPN_INCLUDE_DIR)

libvrpnsink_la_LDFLAGS = \
    $(LIBVRPN_LD_FLAGS)

libvrpnsink_la_CXXFLAGS = \
    $(LIBVRPN_CXX_FLAGS)

libvrpnsink_la_LIBADD = \
    vrpn-lib/build/libvrpnserver.a
