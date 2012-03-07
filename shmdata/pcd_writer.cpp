#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#include "shmdata.h"


#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif



typedef struct _App App;
struct _App
{
    GstElement *pipe;
    GstElement *src;
    GstElement *id;
    gboolean on;
    shmdata::Writer *writer;
};

App s_app;

//clean up pipeline when ctrl-c
void
leave(int sig) {
    s_app.on = false;
    gst_element_set_state (s_app.pipe, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (s_app.pipe));
    exit(sig);
}

int
main (int argc, char** argv)
{

    App *app = &s_app;
    int i;

    (void) signal(SIGINT,leave);

    std::string socketName;

    /* Check input arguments */
    if (argc != 2) {
	g_printerr ("Usage: %s <socket-path>\n", argv[0]);
	return -1;
    }
    socketName.append (argv[1]);
    
    gst_init (&argc, &argv);

    app->pipe = gst_pipeline_new (NULL);
    g_assert (app->pipe);

    app->src = gst_element_factory_make ("appsrc", NULL);
    g_assert (app->src);
    gst_bin_add (GST_BIN (app->pipe), app->src);

    GstCaps *mycaps = gst_caps_new_simple ("application/scenicdata_", NULL);
    gst_app_src_set_caps (GST_APP_SRC(app->src), mycaps);
    //unref ?

    app->id = gst_element_factory_make ("identity", NULL);
    g_assert (app->id);
    gst_bin_add (GST_BIN (app->pipe), app->id);

    app->writer = new shmdata::Writer (app->pipe,app->id,socketName);
    gst_element_link (app->src, app->id);

    app->on = true;
    gst_element_set_state (app->pipe, GST_STATE_PLAYING);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *cloud) == -1) //* load the file
    {
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	return (-1);
    }
    
//    SimpleOpenNIViewer v;
//    v.run (cloud);
    
    pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;


    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    //tunning compression
    bool showStatistics = false;
    double pointResolution = 0.0001;
    double octreeResolution = 0.01;
    bool doVoxelGridDownDownSampling = false;
    unsigned int iFrameRate = 0;
    bool doColorEncoding = true;
    unsigned int colorBitResolution = 8;

    //pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR;//LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;//MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    
    // instantiate point cloud compression for encoding and decoding
    //PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, true);
    PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (pcl::octree::MANUAL_CONFIGURATION, showStatistics, pointResolution, octreeResolution, doVoxelGridDownDownSampling, iFrameRate, doColorEncoding, colorBitResolution);
    // output pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
    

    int mytime=0;
    
    char databuf [500000];
    int usecPeriod=30000;

    while (app->on){  
	// stringstream to store compressed point cloud
	std::stringstream compressedData;

	PointCloudEncoder->encodePointCloud (cloud, compressedData);

	compressedData.read (databuf,500000);
	int size = compressedData.gcount ();
	GstBuffer *gstbuf = gst_app_buffer_new (databuf, size , NULL, NULL);
	GST_BUFFER_TIMESTAMP(gstbuf) = (GstClockTime)((mytime) * 1e9/usecPeriod); 
	gst_app_src_push_buffer (GST_APP_SRC (app->src), gstbuf);
	usleep (usecPeriod);
	mytime++;
    }

    return (0);
}
