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
    check_description_by_class \
    check_fakesink \
    check_quiddity_create_remove \
    check_property_mapper \
    check_property_subscriber \
    check_rtp_session \
    check_signal_subscriber \
    check_startable

check_description_by_class_SOURCES = \
    check_description_by_class.cpp

check_quiddity_create_remove_SOURCES = \
    check_quiddity_create_remove.cpp

check_property_mapper_SOURCES = \
    check_property_mapper.cpp

check_property_subscriber_SOURCES = \
    check_property_subscriber.cpp

check_rtp_session_SOURCES = \
    check_rtp_session.cpp

check_signal_subscriber_SOURCES = \
    check_signal_subscriber.cpp

check_startable_SOURCES = \
    check_startable.cpp

check_fakesink_SOURCES = \
    check_fakesink.cpp

TESTS = $(check_PROGRAMS)
