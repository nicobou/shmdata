#AM_CPPFLAGS = -I$(top_srcdir)/include
AM_CXXFLAGS = -I$(top_srcdir)/include $(GST_CFLAGS) $(GIO_CFLAGS)
AM_LIBS = $(GST_LIBS) $(GIO_LIBS)

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libshmdata-@LIBSHMDATA_API_VERSION@.la

## Define the source file list for the "libshmdata-@SHMDATA_API_VERSION@.la"
## target.  Note that @SHMDATA_API_VERSION@ is not interpreted by Automake and
## will therefore be treated as if it were literally part of the target name,
## and the variable name derived from that.
## The file extension .cc is recognized by Automake, and makes it produce
## rules which invoke the C++ compiler to produce a libtool object file (.lo)
## from each source file.  Note that it is not necessary to list header files
## which are already listed elsewhere in a _HEADERS variable assignment.

## PLEASE KEEP THEM IN ALPHABETICAL ORDER
libshmdata_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    shmdata-reader.cpp \
    shmdata-writer.cpp

## Instruct libtool to include ABI version information in the generated shared
## library file (.so).  The library ABI version is defined in configure.ac, so
## that all version information is kept in one place.
libshmdata_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = $(GST_LIBS) $(GIO_LIBS) -export-dynamic -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_@LIBSHMDATA_API_VERSION@_la_includedir = $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata

libshmdata_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    shmdata.h

