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
    check_bundle \
    check_description_by_class \
    check_gst_pipeline \
    check_information_tree \
    check_manager \
    check_quiddity_create_remove \
    check_rtp_session \
    check_scope_guard \
    check_startable \
    check_test_full \
    check_threaded_wrapper \
    check_ugstelem

check_bundle_SOURCES = \
    check_bundle.cpp

check_description_by_class_SOURCES = \
    check_description_by_class.cpp

check_gst_pipeline_SOURCES = \
    check_gst_pipeline.cpp

check_information_tree_SOURCES = \
    check_information_tree.cpp

check_manager_SOURCES =\
    check_manager.cpp

check_quiddity_create_remove_SOURCES = \
    check_quiddity_create_remove.cpp

check_rtp_session_SOURCES = \
    check_rtp_session.cpp

check_scope_guard_SOURCES = \
    check_scope_guard.cpp

check_startable_SOURCES = \
    check_startable.cpp

check_test_full_SOURCES = \
    check_test_full.cpp

check_threaded_wrapper_SOURCES = \
    check_threaded_wrapper.cpp

check_ugstelem_SOURCES = \
    check_ugstelem.cpp

TESTS = $(check_PROGRAMS)
