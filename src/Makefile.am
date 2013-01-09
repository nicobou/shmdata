bin_PROGRAMS = \
    switcher \
    switcher-ctrl

switcher_SOURCES = \
    main.cpp 

switcher_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

switcher_LDFLAGS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

############# control ##########################

switcher_ctrl_SOURCES = \
   switcher-ctrl.cpp 

switcher_ctrl_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    -I$(top_srcdir) 

switcher_ctrl_LDFLAGS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la


