AM_CFLAGS = \
    -I$(top_srcdir) 

LDADD = \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

check_PROGRAMS = \
    check-sysv-sem \
    check-sysv-shm \
    check-unix-socket

TESTS = $(check_PROGRAMS)

check_sysv_sem_SOURCES = \
    check-sysv-sem.cpp

check_sysv_shm_SOURCES = \
    check-sysv-shm.cpp

check_unix_socket_SOURCES = \
    check-unix-socket.cpp
