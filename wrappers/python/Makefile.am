plugins_LTLIBRARIES = \
	pyshmdata.la

# pyshmdata
pluginsdir = `python3 -c 'from distutils import sysconfig; print(sysconfig.get_python_lib(1,0))'`

pyshmdata_la_SOURCES = \
	pyshmdata.cpp

pyshmdata_la_CXXFLAGS = \
	-std=c++11 \
	$(AM_CXXFLAGS) \
	`python3-config --cflags` \
	-I$(top_builddir)/shmdata

pyshmdata_la_LDFLAGS = \
	-module -fPIC \
	`python3-config --libs` \
	$(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
