plugindir = $(GST_PLUGIN_DIR)

plugin_LTLIBRARIES = libgstshmdata.la

libgstshmdata_la_SOURCES = gstshmdata.c gstshmdatasrc.c gstshmdatasink.c
libgstshmdata_la_CFLAGS = $(GST_CFLAGS)
libgstshmdata_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS)
libgstshmdata_la_LIBADD = $(GST_LIBS) $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la
libgstshmdata_la_LIBTOOLFLAGS = --tag=disable-static

noinst_HEADERS = gstshmdatasrc.h gstshmdatasink.h
