AM_CXXFLAGS = \
    $(GST_CFLAGS) 

AM_LIBS = \
    $(GST_LIBS) 

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    audio-sink.cpp \
    audio-source.cpp \
    base-entity.cpp \
    base-entity-factory.cpp \
    base-sink.cpp \
    base-source.cpp \
    connector.cpp \
    controller.cpp \
    runtime.cpp \
    segment.cpp \
    video-sink.cpp \
    video-source.cpp \
    video-test-source.cpp

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    audio-sink.h \
    audio-source.h \
    base-entity-factory.h \
    base-entity.h \
    base-sink.h \
    base-source.h \
    connector.h \
    controller.h \
    creator.h \
    runtime.h \
    segment.h \
    video-sink.h \
    video-source.h \
    video-test-source.h

