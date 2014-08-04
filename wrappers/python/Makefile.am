python_LTLIBRARIES = \
	pyshmdata.la

# pyshmdata
pyshmdatadir = `python3 -c 'from distutils import sysconfig; print(sysconfig.get_python_lib(1,0))'`

pyshmdata_la_SOURCES = \
	pyshmdata.cpp

pyshmdata_la_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	$(PYTHON3_CFLAGS) \
	-I$(top_builddir)/shmdata

pyshmdata_la_LDFLAGS = \
	-module -fPIC \
	$(PYTHON3_LIBS) \
	$(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la
