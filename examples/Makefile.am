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
    rtp_data_receiver \
    rtp_data_sender \
    video_reader \
    video_writer 

data_reader_LDADD = $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
data_reader_SOURCES = data-reader.c

data_writer_LDADD = $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
data_writer_SOURCES = data-writer.c


rtp_data_sender_CPPFLAGS = $(GSTAPP_CFLAGS)
rtp_data_sender_LDADD = $(LDADD) $(GSTAPP_LIBS) 
rtp_data_sender_SOURCES = rtp-data-sender.c

rtp_data_receiver_SOURCES = rtp-data-receiver.c

video_reader_SOURCES = video-reader.c
video_writer_SOURCES = video-writer.c


