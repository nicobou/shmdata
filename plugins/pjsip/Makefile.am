pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libpjsip_plugin.la 

#libpjsip info
libpjsip_plugin_la_SOURCES = \
    pj-call.cpp \
    pj-codec-utils.cpp \
    pj-codec.cpp \
    pj-presence.cpp \
    pj-sip.cpp 

EXTRA_DIST = \
    more_codecs.patch

nodist_libpjsip_plugin_la_SOURCES = \
    pjproject-2.2.1/install_dir/include/pjsua-lib/pjsua.h

BUILT_SOURCES = \
    pjproject-2.2.1/install_dir/include/pjsua-lib/pjsua.h

libpjsip_plugin_la_include_HEADERS = \
    pj-call.hpp \
    pj-codec.hpp \
    pj-codec-utils.hpp \
    pj-presence.hpp \
    pj-sip.hpp 

libpjsip_plugin_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libpjsip_plugin_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GMODULE_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -L$(top_builddir)/switcher/.libs \
    -avoid-version \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    -module \
    `pkg-config --libs pjproject-2.2.1/install_dir/lib/pkgconfig/libpjproject.pc`

export PJPROJECT_STATIC_LIBS=''

libpjsip_plugin_la_LIBADD = \
    $(PJPROJECT_STATIC_LIBS)

libpjsip_plugin_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    `pkg-config --cflags pjproject-2.2.1/install_dir/lib/pkgconfig/libpjproject.pc` \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) \
    -Wno-error=write-strings \
    -Wno-error=unused-parameter

pjproject-2.2.1/install_dir/include/pjsua-lib/pjsua.h: Makefile
	curl -O http://www.pjsip.org/release/2.2.1/pjproject-2.2.1.tar.bz2 \
        && tar -xjvf pjproject-2.2.1.tar.bz2 \
        && cd pjproject-2.2.1 \
        && mkdir -p install_dir \
        && patch -p0 < ../$(srcdir)/more_codecs.patch \
        && ./configure --prefix `pwd`/install_dir --disable-video  \
           --disable-sound --disable-g711-codec --disable-g7221-codec  \
           --disable-g722-codec --disable-gsm-codec --disable-ilbc-codec \
           --disable-l16-codec --disable-speex-aec  --disable-speex-codec \
        && CFLAGS=-fPIC make -j8 dep \
        && CFLAGS=-fPIC make \
        && make install \
        && PJPROJECT_STATIC_LIBS=`cd ..; ls pjproject-2.2.1/install_dir/lib/*.a`

clean-local:
	-rm -rf pjproject-2.2.1* 

distclean-local:
	-rm -rf pjproject-2.2.1* 

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
    check_pjsip

check_pjsip_SOURCES = \
    check_pjsip.cpp

TESTS = $(check_PROGRAMS)
