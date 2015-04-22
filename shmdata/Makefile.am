AM_CXXFLAGS = \
    -I$(top_srcdir)

lib_LTLIBRARIES = \
    libshmdata-@LIBSHMDATA_API_VERSION@.la 

libshmdata_@LIBSHMDATA_API_VERSION@_la_SOURCES = \
    cfollower.cpp \
    clogger.cpp \
    cwriter.cpp \
    file-monitor.cpp \
    follower.cpp \
    reader.cpp \
    sysv-sem.cpp \
    sysv-shm.cpp \
    unix-socket.cpp \
    unix-socket-client.cpp \
    unix-socket-protocol.cpp \
    unix-socket-server.cpp \
    writer.cpp

libshmdata_@LIBSHMDATA_API_VERSION@_la_include_HEADERS = \
    abstract-logger.hpp \
    cfollower.h \
    clogger.h \
    cwriter.h \
    console-logger.hpp \
    file-monitor.hpp \
    follower.hpp \
    reader.hpp \
    safe-bool-idiom.hpp \
    sysv-sem.hpp \
    sysv-shm.hpp \
    unix-socket.hpp \
    unix-socket-client.hpp \
    unix-socket-protocol.hpp \
    unix-socket-server.hpp \
    writer.hpp

libshmdata_@LIBSHMDATA_API_VERSION@_la_LDFLAGS = \
    -export-dynamic \
    -pthread \
    -version-info $(LIBSHMDATA_SO_VERSION)

libshmdata_@LIBSHMDATA_API_VERSION@_la_includedir = \
    $(includedir)/shmdata-$(LIBSHMDATA_API_VERSION)/shmdata


