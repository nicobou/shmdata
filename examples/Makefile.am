AM_CXXFLAGS = \
    -I$(top_srcdir) \
    $(GST_CFLAGS) \
    $(GIO_CFLAGS)

AM_CFLAGS = \
    $(AM_CXXFLAGS)

LDADD = \
    $(GST_LIBS) \
    $(GIO_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

noinst_PROGRAMS = \
    data_reader \
    data_writer \
    video_reader \
    video_writer 

data_reader_LDADD = $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
data_reader_SOURCES = data-reader.c

data_writer_LDADD = $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
data_writer_SOURCES = data-writer.c

video_reader_SOURCES = video-reader.c
video_writer_SOURCES = video-writer.c


