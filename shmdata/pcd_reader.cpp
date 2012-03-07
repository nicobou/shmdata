#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#include <gst/app/gstappsink.h>
#include "shmdata.h"

#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

typedef struct _App App;
struct _App
{
    GstElement *pipe;
    GstElement *funnel;
    GstElement *sink;
    pcl::visualization::CloudViewer *viewer;
    pcl::octree::PointCloudCompression<pcl::PointXYZRGB> *PointCloudDecoder;
};
 
App s_app;





static void
on_new_buffer_from_source (GstElement * elt, gpointer user_data)
{
    GstBuffer *buf;
    
    /* pull the next item, this can return NULL when there is no more data and
     * EOS has been received */
    buf = gst_app_sink_pull_buffer (GST_APP_SINK (s_app.sink));

    // g_print ("retrieved buffer %p, data %p, data size %d, timestamp %d, caps %s\n", buf, 
    // 	    GST_BUFFER_DATA(buf), GST_BUFFER_SIZE(buf),
    // 	    GST_TIME_AS_MSECONDS(GST_BUFFER_TIMESTAMP(buf)),
    // 	    gst_caps_to_string(GST_BUFFER_CAPS(buf)));
    //g_print ("received: %s\n",GST_BUFFER_DATA(buf));
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // // stringstream to store compressed point cloud
     std::stringstream compressedData;
     compressedData.write ((const char *)GST_BUFFER_DATA(buf),GST_BUFFER_SIZE(buf));
    // // decompress point cloud

     s_app.PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    // // show decompressed point cloud
     s_app.viewer->showCloud (cloudOut);
    
    if (buf)
	gst_buffer_unref (buf);
}




void
on_first_video_data (shmdata::Reader *context, void *user_data)
{
    g_print ("on first data received \n");
    s_app.funnel = gst_element_factory_make ("funnel", NULL);
    g_assert (s_app.funnel);
    s_app.sink = gst_element_factory_make ("appsink", NULL);
    g_assert (s_app.sink);
    g_object_set (G_OBJECT (s_app.sink), "emit-signals", TRUE, "sync", FALSE, NULL);
    g_signal_connect (s_app.sink, "new-buffer",
     		      G_CALLBACK (on_new_buffer_from_source), NULL);
    
    
    gst_element_set_state (s_app.funnel, GST_STATE_PLAYING);
    gst_element_set_state (s_app.sink, GST_STATE_PLAYING);
    gst_bin_add_many (GST_BIN (s_app.pipe), s_app.funnel, s_app.sink, NULL);
    gst_element_link (s_app.funnel, s_app.sink);

    //now tells the shared data reader where to write the data
    context->setSink (s_app.pipe, s_app.funnel);
    
}

void
leave(int sig) {
    gst_element_set_state (s_app.pipe, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (s_app.pipe));
    // delete point cloud compression instances
    delete (s_app.PointCloudDecoder);
    exit(sig);
}


int
main (int argc, char** argv)
{

    GMainLoop *loop;
    
    (void) signal(SIGINT,leave);
    
    s_app.PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();
    s_app.viewer = new pcl::visualization::CloudViewer ("Compressed Point Cloud receiver");

    std::string socketName;
    gst_init (&argc, &argv);
    loop = g_main_loop_new (NULL, FALSE);
    
    if (argc != 2) {
	g_printerr ("Usage: %s <socket-path>\n", argv[0]);
	return -1;
    }
    socketName.append (argv[1]);
    
    s_app.pipe = gst_pipeline_new (NULL);
    g_assert (s_app.pipe);
    
    gst_element_set_state (s_app.pipe, GST_STATE_PLAYING);

    
    shmdata::Reader *reader;
    reader = new shmdata::Reader (socketName, &on_first_video_data,NULL);
    

    g_main_loop_run (loop);

    return 0;
}
