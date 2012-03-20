AM_CFLAGS = -I$(top_srcdir)
LDADD = \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la \
    $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la

check_PROGRAMS = \
    check_write_read

TESTS = $(check_PROGRAMS)

check_write_read_SOURCES = check_write_read.c

