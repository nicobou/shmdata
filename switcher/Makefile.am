AM_CXXFLAGS = \
    $(GST_CFLAGS) 

AM_LIBS = \
    $(GST_LIBS) 

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    base-entity.cpp \
    controller.cpp \
    runtime.cpp 

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    base-entity.h \
    controller.h \
    runtime.h 
