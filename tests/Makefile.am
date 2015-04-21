AM_CFLAGS = \
    -I$(top_srcdir) 

LDADD = \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

check_PROGRAMS = \
    check-file-monitor \
    check-follower \
    check-shmdata \
    check-shmdata-stress \
    check-sysv-sem \
    check-sysv-shm \
    check-unix-socket

TESTS = $(check_PROGRAMS)

check_file_monitor_SOURCES = \
    check-file-monitor.cpp

check_follower_SOURCES = \
    check-follower.cpp

check_shmdata_SOURCES = \
    check-shmdata.cpp

check_shmdata_stress_SOURCES = \
    check-shmdata-stress.cpp

check_sysv_sem_SOURCES = \
    check-sysv-sem.cpp

check_sysv_shm_SOURCES = \
    check-sysv-shm.cpp

check_unix_socket_SOURCES = \
    check-unix-socket.cpp
