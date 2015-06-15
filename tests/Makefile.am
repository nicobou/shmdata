AM_CXXFLAGS = \
    -I$(top_srcdir) 

LDADD = \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

check_PROGRAMS = \
    check-c-wrapper \
    check-file-monitor \
    check-follower \
    check-shmdata \
    check-shmdata-stress \
    check-sysv-sem \
    check-sysv-shm \
    check-unix-socket \
    check-writer-follower

TESTS = $(check_PROGRAMS)

check_c_wrapper_SOURCES = \
    check-c-wrapper.cpp

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

check_writer_follower_SOURCES = \
    check-writer-follower.cpp
