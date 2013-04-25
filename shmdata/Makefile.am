AM_CFLAGS = \
    -I$(top_srcdir) \
    -DG_LOG_DOMAIN=\"shmdata\" \
    $(GST_CFLAGS) \
    $(GIO_CFLAGS)

AM_LIBS = \
    $(GST_LIBS) \
    $(GIO_LIBS)

# if HAVE_OSX
#  AM_CFLAGS += -arch i386 -arch x86_64 -mmacosx-version-min=10.4 -ftree-vectorize  -fPIC 
#  AM_LDFLAGS = -arch i386 -arch x86_64 -mmacosx-version-min=10.4 -bundle -undefined dynamic_lookup
# endif

lib_LTLIBRARIES = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la \
    libshmdata-any-@LIBSHMDATA_API_VERSION@.la 

## base library
libshmdata_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    base-reader.c \
    base-writer.c

libshmdata_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    $(AM_LDFLAGS) \
    $(GIO_LIBS) \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

libshmdata_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    base-reader.h \
    base-writer.h


## openscenegraph library
if HAVE_OPENSCENEGRAPH
lib_LTLIBRARIES += \
    libshmdata-osg-@LIBSHMDATA_API_VERSION@.la 

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    osg-reader.h \
    osg-reader.cpp \
    osg-reader_impl.cpp \
    osg-reader_impl.h 

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_CXXFLAGS = \
    $(AM_CFLAGS) \
    $(GSTAPP_CFLAGS) \
    $(OPENSCENEGRAPH_CFLAGS) 

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    $(AM_LDFLAGS) \
    $(GSTAPP_LIBS) \
    $(OPENSCENEGRAPH_LIBS) \
    -export-dynamic \
    -version-info $(LIBSHMDATA_SO_VERSION) 

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_LIBADD = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    osg-reader.h 

endif

## any data library
libshmdata_any_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    any-data-reader.c \
    any-data-writer.c

libshmdata_any_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    $(AM_LDFLAGS) \
    $(GSTAPP_LIBS) \
    -export-dynamic \
    -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_any_@LIBSHMDATA_API_VERSION@_la_LIBADD = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la

libshmdata_any_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

libshmdata_any_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    any-data-reader.h \
    any-data-writer.h 

