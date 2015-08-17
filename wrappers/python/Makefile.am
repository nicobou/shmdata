plugins_LTLIBRARIES = \
	pyshmdata.la

# pyshmdata
pluginsdir = ${prefix}`python3.4 -c 'import sys; from distutils import sysconfig; print((sysconfig.get_python_lib(1,0)).split(sys.prefix)[1])'`

pyshmdata_la_SOURCES = \
    pyshmdata.cpp

noinst_HEADERS = \
    pyshmdata.h

noinst_HEADERS = \
	pyshmdata.h

pyshmdata_la_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	`python3.4-config --cflags` \
	-I$(top_srcdir)

pyshmdata_la_LDFLAGS = \
	-module -fPIC \
	`python3.4-config --ldflags` \
	$(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la

if HAVE_OSX
pyshmdata_la_LDFLAGS += -L/opt/local/lib
endif
