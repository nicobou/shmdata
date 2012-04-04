AM_CFLAGS = \
    $(GST_CFLAGS) 

AM_LIBS = \
    $(GST_LIBS) 

lib_LTLIBRARIES = \
    libswitcher-live-input-@LIBSWITCHER_LIVE_INPUT_API_VERSION@.la

libswitcher_live_input_@LIBSWITCHER_LIVE_INPUT_API_VERSION@_la_SOURCES = \
    switcher-live-input.c 

libswitcher_live_input_@LIBSWITCHER_LIVE_INPUT_API_VERSION@_la_LDFLAGS = \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSWITCHER_LIVE_INPUT_SO_VERSION)

libswitcher_live_input_@LIBSWITCHER_LIVE_INPUT_API_VERSION@_la_includedir = \
    $(includedir)/switcher-live-input-$(LIBSWITCHER_LIVE_INPUT_API_VERSION)/switcher-live-input

libswitcher_live_input_@LIBSWITCHER_LIVE_INPUT_API_VERSION@_la_include_HEADERS = \
    switcher-live-input.h 


