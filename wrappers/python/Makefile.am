plugins_LTLIBRARIES = \
	pyshmdata.la

# pyshmdata
pluginsdir = ${prefix}`python3 -c 'import sys; from distutils import sysconfig; print((sysconfig.get_python_lib(1,0)).split(sys.prefix)[1])'`

pyshmdata_la_SOURCES = \
	pyshmdata.cpp

noinst_HEADERS = \
	pyshmdata.h

pyshmdata_la_CXXFLAGS = \
	-std=c++11 \
	$(AM_CXXFLAGS) \
	`python3-config --cflags` \
	-I$(top_srcdir)

pyshmdata_la_LDFLAGS = \
	-module -fPIC \
	`python3-config --libs` \
	$(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
