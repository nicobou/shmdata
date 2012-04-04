AM_CFLAGS = \
    $(GST_CFLAGS) 

AM_LIBS = \
    $(GST_LIBS) 

lib_LTLIBRARIES = \
    libswitcher-stored-input-@LIBSWITCHER_STORED_INPUT_API_VERSION@.la

libswitcher_stored_input_@LIBSWITCHER_STORED_INPUT_API_VERSION@_la_SOURCES = \
    switcher-stored-input.c 

libswitcher_stored_input_@LIBSWITCHER_STORED_INPUT_API_VERSION@_la_LDFLAGS = \
    $(GST_LIBS) \
    -export-dynamic \
    -version-info $(LIBSWITCHER_STORED_INPUT_SO_VERSION)

libswitcher_stored_input_@LIBSWITCHER_STORED_INPUT_API_VERSION@_la_includedir = \
    $(includedir)/switcher-stored-input-$(LIBSWITCHER_STORED_INPUT_API_VERSION)/switcher-stored-input

libswitcher_stored_input_@LIBSWITCHER_STORED_INPUT_API_VERSION@_la_include_HEADERS = \
    switcher-stored-input.h 


