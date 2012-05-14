bin_PROGRAMS = \
    switcher

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
