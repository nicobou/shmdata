plugindir = \
    $(SHMDATA_GST_PLUGIN_PATH)

plugin_LTLIBRARIES = \
    libgstshm.la 

libgstshm_la_SOURCES = \
    gstshm.c \
    gstshmsink.c \
    gstshmsrc.c \
    shmalloc.c \
    shmpipe.c

libgstshm_la_CFLAGS = \
    $(GST_CFLAGS) \
    -DSHM_PIPE_USE_GLIB

if HAVE_OSX
 libgstshm_la_LIBADD =
else
 libgstshm_la_LIBADD = -lrt
endif

#from Manu's gst-plugin-bad fork 
libgstshm_la_LDFLAGS = \
    $(GST_BASE_LIBS) \
    -module \
    -avoid-version \
    -export-symbols-regex '^[_]*gst_plugin_desc.*' \
    -no-undefined

libgstshm_la_LIBTOOLFLAGS = \
    --tag=disable-static

noinst_HEADERS = \
    gstshmsink.h \
    gstshmsrc.h \
    shmalloc.h \
    shmpipe.h
