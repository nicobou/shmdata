AM_CXXFLAGS = \
    -I$(top_srcdir)/shmdata/ \
    $(GST_CFLAGS) $(GIO_CFLAGS)

AM_CFLAGS = \
    $(AM_CXXFLAGS)

LDADD = \
    $(GST_LIBS) \
    $(GIO_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

noinst_PROGRAMS = \
    data_reader \
    data_reader_from_rtp \
    data_reader_to_rtp \
    data_writer \
    data_writer_from_rtp \
    video_reader \
    video_writer

if BUILD_CLUTTER_EXAMPLE 
    noinst_PROGRAMS += \
        clutter_reader
endif

if HAVE_OPENSCENEGRAPH
    noinst_PROGRAMS += \
        osg_reader
endif

if HAVE_PCL
    noinst_PROGRAMS += \
        pcd_reader \
        pcd_writer 
endif

clutter_reader_CPPFLAGS = $(CLUTTER_CFLAGS) $(GLEW_CFLAGS)
clutter_reader_LDADD = $(LDADD) $(CLUTTER_LIBS) $(GLEW_LIBS) 
clutter_reader_SOURCES = clutter-actor-reader.c

data_reader_CPPFLAGS = $(GSTAPP_CFLAGS)
data_reader_LDADD = $(LDADD) $(GSTAPP_LIBS) 
data_reader_SOURCES = data-reader.c

data_reader_from_rtp_CPPFLAGS = $(GSTAPP_CFLAGS)
data_reader_from_rtp_LDADD = $(LDADD) $(GSTAPP_LIBS) 
data_reader_from_rtp_SOURCES = data-reader-from-rtp.c

data_reader_to_rtp_CPPFLAGS = $(GSTAPP_CFLAGS)
data_reader_to_rtp_LDADD = $(LDADD) $(GSTAPP_LIBS) 
data_reader_to_rtp_SOURCES = data-reader-to-rtp.c

data_writer_CPPFLAGS = $(GSTAPP_CFLAGS)
data_writer_LDADD = $(GSTAPP_LIBS) $(LDADD) 
data_writer_SOURCES = data-writer.c

data_writer_from_rtp_SOURCES = data-writer-from-rtp.c

osg_reader_CPPFLAGS = $(OPENSCENEGRAPH_CFLAGS) 
osg_reader_LDADD = $(OPENSCENEGRAPH_LIBS) $(top_builddir)/shmdata/libshmdata-osg-@LIBSHMDATA_API_VERSION@.la 
osg_reader_SOURCES = osg-reader.cpp

pcd_reader_CPPFLAGS = $(PCL_CFLAGS) -I /usr/include/vtk-5.6/ $(GSTAPP_CFLAGS)
pcd_reader_LDADD = $(PCL_LIBS) $(GSTAPP_LIBS) $(LDADD) 
pcd_reader_SOURCES = pcd_reader.cpp

pcd_writer_CPPFLAGS = $(PCL_CFLAGS) -I /usr/include/vtk-5.6/ $(GSTAPP_CFLAGS)
pcd_writer_LDADD = $(LDADD) $(PCL_LIBS) $(GSTAPP_LIBS) 
pcd_writer_SOURCES = pcd_writer.cpp

video_reader_SOURCES = video-reader.c
video_writer_SOURCES = video-writer.c

