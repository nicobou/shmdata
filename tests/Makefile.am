export GST_PLUGIN_PATH=$(top_builddir)/gst/shm:$GST_PLUGIN_PATH

AM_CFLAGS = -I$(top_srcdir)

LDADD = \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la \
    $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la

#LD_LIBRARY_PATH = \
#    $(top_builddir)/gst/shm/

check_PROGRAMS = \
    check_read_then_write \
    check_read_and_write_onoff \
    check_write_read \
    check_write_and_read_onoff

TESTS = $(check_PROGRAMS)

check_write_read_SOURCES = \
    check_write_read.c

check_read_then_write_SOURCES = \
    check_read_then_write.c

check_read_and_write_onoff_SOURCES = \
    check_read_and_write_onoff.c

check_write_and_read_onoff_SOURCES = \
    check_write_and_read_onoff.c
