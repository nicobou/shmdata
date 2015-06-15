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
