AM_CFLAGS = \
    -I$(top_srcdir) 

LDADD = \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

check_PROGRAMS = \
    check_sysv_shm 

TESTS = $(check_PROGRAMS)

check_sysv_shm_SOURCES = \
    check_sysv_shm.cpp
