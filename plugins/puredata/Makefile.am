# #
pddir = $(prefix)/lib/pd
plugindir = $(pddir)/extra

AM_CFLAGS = \
    -I$(top_srcdir) \
    $(GST_CFLAGS) \
    $(GIO_CFLAGS) \
    $(LIBSAMPLERATE_CFLAGS)

AM_LDFLAGS =  \
    $(LIBSAMPLERATE_LIBS) \
    -avoid-version \
    -export-dynamic \
    -fPIC \
    -module \
    -shared 

if HAVE_OSX
 AM_LDFLAGS += -shrext .pd_darwin
# AM_CFLAGS += -arch i386 -arch x86_64
else
 AM_LDFLAGS += -shrext .pd_linux 
endif


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
    $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la

shmsrc__la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

shmsrc__la_SOURCES = \
    shmsrc~.c


