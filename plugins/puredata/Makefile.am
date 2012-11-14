# #
pddir = $(prefix)/lib/pd
plugindir = $(pddir)/extra

AM_CFLAGS = \
    -I$(top_srcdir) \
    $(GST_CFLAGS) \
    $(GIO_CFLAGS)


AM_LDFLAGS =  \
    -module \
    -avoid-version \
    -shared \
    -shrext .pd_linux \
    -export-dynamic \
    -fPIC

plugin_LTLIBRARIES = \
    shmsink~.la \
    shmsrc~.la

##sink
shmsink__la_LIBADD = \
    $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
shmsink__la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata
shmsink__la_SOURCES = \
    shmsink~.c

##src
shmsrc__la_LIBADD = \
    $(GST_LIBS) \
    $(GIO_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la
shmsrc__la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata
shmsrc__la_SOURCES = \
    shmsrc~.c


