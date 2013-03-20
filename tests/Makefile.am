AM_CFLAGS = \
    -I$(top_srcdir)

check_PROGRAMS = \
    check_quiddity_create_remove \
    check_quiddity_create_remove_with_auto_pipeline \
    check_property_subscriber

TESTS = $(check_PROGRAMS)


##-------------- check_quiddity_create_remove
check_quiddity_create_remove_SOURCES = \
    check_quiddity_create_remove.cpp

check_quiddity_create_remove_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

check_quiddity_create_remove_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

##-------------- check_quiddity_create_remove_with_auto_pipeline
check_quiddity_create_remove_with_auto_pipeline_SOURCES = \
    check_quiddity_create_remove_with_auto_pipeline.cpp

check_quiddity_create_remove_with_auto_pipeline_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

check_quiddity_create_remove_with_auto_pipeline_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

##-------------- check_property_subscriber
check_property_subscriber_SOURCES = \
    check_property_subscriber.cpp

check_property_subscriber_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

check_property_subscriber_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la
