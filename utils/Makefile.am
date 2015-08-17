AM_CXXFLAGS = \
    -I$(top_srcdir)

AM_CFLAGS = \
    $(AM_CXXFLAGS)

bin_PROGRAMS = \
    sdflow \
    sdcrash

sdflow_LDADD = $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la
sdflow_SOURCES = sdflow.cpp

sdcrash_LDADD = $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la
sdcrash_SOURCES = sdcrash.cpp



