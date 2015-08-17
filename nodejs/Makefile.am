pluginsdir = $(prefix)/lib/nodejs

plugins_LTLIBRARIES = \
    switcher.la

#libplugin info
switcher_la_SOURCES = \
    switcher-controller.cpp \
    switcher-addon.cpp

switcher_la_include_HEADERS = \
    switcher-controller.hpp

switcher_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

switcher_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GMODULE_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -L$(top_builddir)/switcher/.libs \
    -avoid-version \
    -bundle \
    -flat_namespace \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    -module \
    -shrext .node \
    -undefined suppress 

switcher_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DEV_MULTIPLICITY=0 \
    -DG_LOG_DOMAIN=\"switcher\" \
    -DPIC \
    -D_GNU_SOURCE \
    -D_LARGEFILE_SOURCE \
    -I$(top_srcdir) \
    -I/usr/include/nodejs/deps/uv/include/ \
    -I/usr/include/nodejs/src \
    -fPIC

#    -D_FILE_OFFSET_BITS=64 \
#    -m64 

