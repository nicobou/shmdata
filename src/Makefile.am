bin_PROGRAMS = \
    switcher \
    switcher-ctrl

switcher_SOURCES = \
   main.cpp 

# noinst_HEADERS = \
#    gst-app.h \
#    play.h

switcher_CXXFLAGS = \
    $(GST_CFLAGS) \
    -I$(top_srcdir) 

switcher_LDFLAGS = \
    $(GST_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

############# control ##########################

switcher_ctrl_SOURCES = \
   switcher-ctrl.cpp 

switcher_ctrl_CXXFLAGS = \
    -I$(top_srcdir) 

switcher_ctrl_LDFLAGS = \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la


