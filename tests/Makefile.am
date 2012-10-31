AM_CFLAGS = \
    -I$(top_srcdir)

check_PROGRAMS = \
    check_quiddity_create_remove

TESTS = $(check_PROGRAMS)

check_quiddity_create_remove_SOURCES = \
    check_quiddity_create_remove.cpp

check_quiddity_create_remove_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

check_quiddity_create_remove_LDFLAGS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la
