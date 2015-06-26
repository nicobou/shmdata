plugindir = \
    $(GST_PLUGIN_DIR)

plugin_LTLIBRARIES = \
    libgstshmdata.la

libgstshmdata_la_SOURCES = \
    gstshmdata.c \
    gstshmdatasink.c \
    gstshmdatasrc.c

libgstshmdata_la_CFLAGS = \
    $(GST_CFLAGS) \
    -I$(top_srcdir)

libgstshmdata_la_LDFLAGS = \
    $(GST_PLUGIN_LDFLAGS)

libgstshmdata_la_LIBADD = \
    $(GST_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la

libgstshmdata_la_LIBTOOLFLAGS = \
    --tag=disable-static

noinst_HEADERS = \
    gstshmdatalogger.h \
    gstshmdatasink.h \
    gstshmdatasrc.h


check_PROGRAMS = \
    check-shmdatasrc \
    check-shmdatasink

TESTS = $(check_PROGRAMS)

check_shmdatasrc_SOURCES = \
    check-shmdatasrc.c

check_shmdatasrc_CFLAGS = \
    $(GST_CFLAGS) \
    -I$(top_srcdir)

check_shmdatasrc_LDADD = \
    $(GST_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la

check_shmdatasink_SOURCES = \
    check-shmdatasink.c

check_shmdatasink_CFLAGS = \
    $(GST_CFLAGS) \
    -I$(top_srcdir)

check_shmdatasink_LDADD = \
    $(GST_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la

