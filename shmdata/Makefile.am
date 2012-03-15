AM_CFLAGS = \
    -DG_LOG_DOMAIN=\"shmdata\" \
    $(GST_CFLAGS) \
    $(GIO_CFLAGS)

AM_LIBS = \
    $(GST_LIBS) \
    $(GIO_LIBS)

lib_LTLIBRARIES = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la \
    libshmdata-osg-@LIBSHMDATA_API_VERSION@.la 

## core library
libshmdata_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    reader.c \
    writer.c

libshmdata_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    $(GIO_LIBS) \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

libshmdata_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    reader.h \
    writer.h

## openscenegraph library
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
    $(GIO_LIBS) \
    $(GSTAPP_LIBS) \
    $(GST_LIBS) \
    $(OPENSCENEGRAPH_LIBS) \
    -export-dynamic \
    -lshmdata-@LIBSHMDATA_API_VERSION@ \
    -version-info $(LIBSHMDATA_SO_VERSION) 

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

libshmdata_osg_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    osg-reader.h 

