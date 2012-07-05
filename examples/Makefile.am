AM_CXXFLAGS = \
    -I$(top_srcdir) \
    $(GST_CFLAGS) $(GIO_CFLAGS)

AM_CFLAGS = \
    $(AM_CXXFLAGS)

LDADD = \
    $(GST_LIBS) \
    $(GIO_LIBS) \
    $(top_builddir)/shmdata/libshmdata-@LIBSHMDATA_API_VERSION@.la 

noinst_PROGRAMS = \
    data_reader \
    data_writer \
    rtp_data_receiver \
    rtp_data_sender \
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

#data_reader_CPPFLAGS = $(GSTAPP_CFLAGS)
data_reader_LDADD = $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
data_reader_SOURCES = data-reader.c

rtp_data_sender_CPPFLAGS = $(GSTAPP_CFLAGS)
rtp_data_sender_LDADD = $(LDADD) $(GSTAPP_LIBS) 
rtp_data_sender_SOURCES = rtp-data-sender.c

data_writer_LDADD = $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
data_writer_SOURCES = data-writer.c

rtp_data_receiver_SOURCES = rtp-data-receiver.c

osg_reader_CPPFLAGS = $(OPENSCENEGRAPH_CFLAGS) 
osg_reader_LDADD = $(OPENSCENEGRAPH_LIBS) $(top_builddir)/shmdata/libshmdata-osg-@LIBSHMDATA_API_VERSION@.la 
osg_reader_SOURCES = osg-reader.cpp

pcd_reader_CPPFLAGS = -Wno-deprecated $(PCL_CFLAGS) -I /usr/include/vtk-5.8/ 
#next line commented because of bugy pc file in libflann
#pcd_reader_LDADD = $(PCL_LIBS) $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
pcd_reader_LDADD = -lpcl_visualization -lpcl_io -lpcl_kdtree -lOpenNI -lpcl_octree -lusb-1.0 -lflann_cpp -lpcl_common  $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la

pcd_reader_SOURCES = pcd_reader.cpp

pcd_writer_CPPFLAGS = $(PCL_CFLAGS) -I /usr/include/vtk-5.8/ 
#next line commented because of bugy pc file in libflann
#pcd_writer_LDADD = $(PCL_LIBS) $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
pcd_writer_LDADD =  -lpcl_visualization -lpcl_io -lpcl_kdtree -lOpenNI -lpcl_octree -lusb-1.0 -lflann_cpp -lpcl_common $(top_builddir)/shmdata/libshmdata-any-@LIBSHMDATA_API_VERSION@.la
pcd_writer_SOURCES = pcd_writer.cpp

video_reader_SOURCES = video-reader.c
video_writer_SOURCES = video-writer.c

