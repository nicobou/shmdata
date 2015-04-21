plugindir = $(GST_PLUGIN_DIR)

plugin_LTLIBRARIES = libgstshmdata.la

libgstshmdata_la_SOURCES = shmpipe.c shmalloc.c gstshmdata.c gstshmdatasrc.c gstshmdatasink.c
libgstshmdata_la_CFLAGS = $(GST_CFLAGS) -DSHM_PIPE_USE_GLIB
libgstshmdata_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS) -lrt
libgstshmdata_la_LIBADD = $(GST_LIBS)
libgstshmdata_la_LIBTOOLFLAGS = --tag=disable-static

noinst_HEADERS = gstshmdatasrc.h gstshmdatasink.h shmpipe.h  shmalloc.h
