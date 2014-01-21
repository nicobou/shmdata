bin_PROGRAMS = \
    switcher \
    switcher-ctrl

switcher_SOURCES = \
    main.cpp 

switcher_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

switcher_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

#use gtk loop in order to block main (required for gtk working in OSX)
if GTKBUILD
switcher_CXXFLAGS += \
    $(GTK_CFLAGS)

switcher_LDFLAGS += \
    $(GTK_LIBS)
endif


############# control ##########################

switcher_ctrl_SOURCES = \
   switcher-ctrl.cpp 

switcher_ctrl_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    -I$(top_srcdir) \
    -I$(top_builddir)/plugins/gsoap/

switcher_ctrl_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(top_builddir)/plugins/gsoap/soapC.o \
    $(top_builddir)/plugins/gsoap/soapcontrolProxy.o
