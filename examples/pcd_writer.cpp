/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

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
#include "shmdata/any-data-writer.h"

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

static gchar *socket_path = "/tmp/pcd_to_read";
static gchar *pcd_file = "test_pcd.pcd";
static gboolean verbose = false;

static double framerate = 30.;
static bool showStatistics = false;
static double pointResolution = 0.0001;
static double octreeResolution = 0.01;
static bool doVoxelGridDownDownSampling = false;
//It is critical to keep iFrame to 0 in order to get the decoder always starting with a key frame 
static unsigned int iFrameRate = 0;
static bool doColorEncoding = true;
static unsigned int colorBitResolution = 8;


 static GOptionEntry entries[] =
 {

   { "socket-path", 's', 0, G_OPTION_ARG_STRING, &socket_path, "socket path for the shmdata (default /tmp/pcd_to_read)", NULL },
   { "pcd-file", 'f', 0, G_OPTION_ARG_STRING, &pcd_file, "the file to write (test_pcd.pcd)", NULL },
   { "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "enable printing of messages", NULL },
   { "frame-rate", 'r', 0, G_OPTION_ARG_DOUBLE, &framerate, "frame writing rate (30) ", NULL },
   { "show-stats", 'S', 0, G_OPTION_ARG_NONE, &showStatistics, "enable statistics from compression", NULL },
   { "point-res", 'P', 0, G_OPTION_ARG_DOUBLE, &pointResolution, "point resolution (0.0001)", NULL },
   { "octree-res", 'O', 0, G_OPTION_ARG_DOUBLE, &octreeResolution, "octree resolution (0.01)", NULL },
   { "down-sampling", 'D', 0, G_OPTION_ARG_NONE, &doVoxelGridDownDownSampling, "enable voxel grid down sampling", NULL },
   { "i-frame-rate", 'R', 0, G_OPTION_ARG_INT, &iFrameRate, "I-frame rate (0) ", NULL },
   //{ "no-color-encoding", 'E', 0, G_OPTION_ARG_NONE, &doColorEncoding, "disable color encoding", NULL },
   //{ "color-bit-resolution", 'C', 0, G_OPTION_ARG_INT, &colorBitResolution, "color resultion in bits (8)", NULL },
   { NULL, NULL, 0, G_OPTION_ARG_NONE, NULL, NULL, NULL }
 };
 
typedef struct _App App;
struct _App
{
  gboolean on;
  shmdata_any_writer_t *writer;
};

App s_app;

//clean up pipeline when ctrl-c
void
leave (int sig)
{
  s_app.on = false;
//    exit(sig);
}

int
main (int argc, char *argv[])
{

  //command line options
  GError *error = NULL;
  GOptionContext *context;
  context = g_option_context_new ("- read a pcd file, compress it and write it as a data stream to a shmdata-any. Compression is operated for each frame.");
  g_option_context_add_main_entries (context, entries, NULL);
  if (!g_option_context_parse (context, &argc, &argv, &error))
    {
      g_print ("option parsing failed: %s\n", error->message);
      exit (1);
    } 

  (void) signal (SIGINT, leave);

  //shmdata
  s_app.writer = shmdata_any_writer_init ();
  if (verbose)
    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_ENABLE_DEBUG);
  else
    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_DISABLE_DEBUG);
 
  if (verbose)
    g_print ("setting data type to application/x-pcd\n");
  shmdata_any_writer_set_data_type (s_app.writer, "application/x-pcd");
  
  if (verbose)
    g_print ("set shared memory path to %s\n",socket_path);
      
  if (! shmdata_any_writer_set_path (s_app.writer,socket_path))
    {
      g_printerr ("**** The file %s exists, therefore a shmdata cannot be operated with this path.\n",socket_path);	
      shmdata_any_writer_close (s_app.writer);
      exit(0);
    }

if (verbose)
    g_print ("writing now\n",socket_path);

  shmdata_any_writer_start (s_app.writer);

  //---- reading a pcd file
  pcl::PointCloud < pcl::PointXYZRGB >::Ptr cloud (new pcl::PointCloud <
						   pcl::PointXYZRGB >);
  //loading the cloud
  if (verbose)
    g_print ("loading once the pcd file %s\n",pcd_file);

  if (pcl::io::loadPCDFile < pcl::PointXYZRGB > (pcd_file, *cloud) ==
      -1)
    {
      PCL_ERROR ("Couldn't read the pcd file\n");
      return (-1);
    }

  //---- compression
  pcl::octree::PointCloudCompression < pcl::PointXYZRGB > *PointCloudEncoder;


  if(verbose)
    g_print ("instancating the compressor\n");

  PointCloudEncoder =
    new pcl::octree::PointCloudCompression < pcl::PointXYZRGB >
    (pcl::octree::MANUAL_CONFIGURATION, showStatistics, pointResolution,
     octreeResolution, doVoxelGridDownDownSampling, iFrameRate,
     doColorEncoding, colorBitResolution);

  char databuf[500000];
  unsigned long long myclock = 0;
  unsigned long long nsecPeriod = (unsigned long long) (1000000000. / (double)framerate);

  s_app.on = true;


  //used for printing
  int initcount = (int)framerate;
  if (initcount == 0) 
    initcount = 1;
  int count = initcount;


  while (s_app.on)
    {
      // stringstream to store compressed point cloud
      std::stringstream compressedData;
      PointCloudEncoder->encodePointCloud (cloud, compressedData);

      compressedData.read (databuf, 500000);
      int size = compressedData.gcount ();

      shmdata_any_writer_push_data (s_app.writer,
				    databuf, size, myclock, NULL, NULL);
      
      //basic clock implementation
      usleep (nsecPeriod / 1000);
      myclock += nsecPeriod;
      
      if (verbose)
	{
	  count = count - 1;
	  if (count == 0)
	    {
	      count = initcount;
	      g_print ("%d new frames has been writen (framerate is %f)\n",initcount,framerate);
	    }
	}
    }

  //s_app.on is false
  shmdata_any_writer_close (s_app.writer);
  return (0);
}

