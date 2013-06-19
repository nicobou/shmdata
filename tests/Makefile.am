AM_CFLAGS = \
    -I$(top_srcdir)

AM_CXXFLAGS =  \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

AM_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

check_PROGRAMS = \
    check_quiddity_create_remove \
    check_quiddity_create_remove_with_auto_pipeline \
    check_property_subscriber \
    check_rtp_session \
    check_signal_subscriber

check_quiddity_create_remove_SOURCES = \
    check_quiddity_create_remove.cpp

check_quiddity_create_remove_with_auto_pipeline_SOURCES = \
    check_quiddity_create_remove_with_auto_pipeline.cpp

check_property_subscriber_SOURCES = \
    check_property_subscriber.cpp

check_rtp_session_SOURCES = \
    check_rtp_session.cpp

check_signal_subscriber_SOURCES = \
    check_signal_subscriber.cpp

TESTS = $(check_PROGRAMS)
