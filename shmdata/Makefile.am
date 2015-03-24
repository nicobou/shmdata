AM_CXXFLAGS = \
    -I$(top_srcdir)

lib_LTLIBRARIES = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la 

libshmdata_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    sysv-shm.cpp \
    unix-socket.cpp \
    unix-socket-client.cpp \
    unix-socket-protocol.cpp \
    unix-socket-server.cpp

libshmdata_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    safe-bool-idiom.hpp \
    sysv-shm.hpp \
    unix-socket.hpp \
    unix-socket-client.hpp \
    unix-socket-protocol.hpp \
    unix-socket-server.hpp

libshmdata_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    -export-dynamic \
    -pthread \
    -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata


