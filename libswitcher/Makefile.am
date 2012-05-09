AM_CFLAGS = \
    $(GST_CFLAGS) 

AM_LIBS = \
    $(GST_LIBS) 

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    switcher-stored-input.c 

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    switcher-stored-input.h 


