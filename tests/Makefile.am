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

# check_PROGRAMS = \
#     check_fake_shmdata_writer \
#     check_property_mapper \
#     check_property_subscriber \
#     check_signal_subscriber


check_PROGRAMS = \
    check_description_by_class \
    check_gst_pipeline \
    check_information_tree \
    check_make_consultable \
    check_manager \
    check_quiddity_create_remove \
    check_rtp_session \
    check_scope_guard \
    check_startable \
    check_string_dictionary \
    check_ugstelem

check_description_by_class_SOURCES = \
    check_description_by_class.cpp

check_fake_shmdata_writer_SOURCES = \
    check_fake_shmdata_writer.cpp

check_gst_pipeline_SOURCES = \
    check_gst_pipeline.cpp

check_information_tree_SOURCES = \
    check_information_tree.cpp

check_make_consultable_SOURCES = \
    check_make_consultable.cpp

check_manager_SOURCES =\
    check_manager.cpp

check_property_mapper_SOURCES = \
    check_property_mapper.cpp

check_property_subscriber_SOURCES = \
    check_property_subscriber.cpp

check_quiddity_create_remove_SOURCES = \
    check_quiddity_create_remove.cpp

check_rtp_session_SOURCES = \
    check_rtp_session.cpp

check_scope_guard_SOURCES = \
    check_scope_guard.cpp

check_signal_subscriber_SOURCES = \
    check_signal_subscriber.cpp

check_startable_SOURCES = \
    check_startable.cpp

check_string_dictionary_SOURCES = \
    check_string_dictionary.cpp

check_ugstelem_SOURCES = \
    check_ugstelem.cpp

TESTS = $(check_PROGRAMS)
