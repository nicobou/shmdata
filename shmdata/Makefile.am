AM_CXXFLAGS = \
    -I$(top_srcdir)

lib_LTLIBRARIES = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la 

libshmdata_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    sysv-shm.cpp \
    unix-socket.cpp

libshmdata_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    safe-bool-idiom.hpp \
    sysv-shm.hpp \
    unix-socket.hpp

libshmdata_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    -export-dynamic \
    -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata


