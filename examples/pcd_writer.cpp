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

  (void) signal (SIGINT, leave);

  /* Check input arguments */
  if (argc != 2)
    {
      g_printerr ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
    }

  //shmdata
  s_app.writer = shmdata_any_writer_init ();
  shmdata_any_writer_set_debug (s_app.writer, SHMDATA_ENABLE_DEBUG);
  shmdata_any_writer_set_data_type (s_app.writer, "application/x-pcd");
  shmdata_any_writer_start (s_app.writer, argv[1]);

  //---- reading a pcd file
  pcl::PointCloud < pcl::PointXYZRGB >::Ptr cloud (new pcl::PointCloud <
						   pcl::PointXYZRGB >);
  //loading the cloud
  if (pcl::io::loadPCDFile < pcl::PointXYZRGB > ("test_pcd.pcd", *cloud) ==
      -1)
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

  //---- compression
  pcl::octree::PointCloudCompression < pcl::PointXYZRGB > *PointCloudEncoder;
  //tunning compression for rtp streaming:
  //     require iFrameRate to 0, otherwise the remote decoder is freezing
  bool showStatistics = false;
  double pointResolution = 0.0001;
  double octreeResolution = 0.01;
  bool doVoxelGridDownDownSampling = false;
  unsigned int iFrameRate = 0;
  bool doColorEncoding = true;
  unsigned int colorBitResolution = 8;
  // instantiate point cloud compression for encoding and decoding
  PointCloudEncoder =
    new pcl::octree::PointCloudCompression < pcl::PointXYZRGB >
    (pcl::octree::MANUAL_CONFIGURATION, showStatistics, pointResolution,
     octreeResolution, doVoxelGridDownDownSampling, iFrameRate,
     doColorEncoding, colorBitResolution);

  char databuf[500000];
  unsigned long long myclock = 0;
  unsigned long long nsecPeriod = 30000000;

  s_app.on = true;

  while (s_app.on)
    {
      // stringstream to store compressed point cloud
      std::stringstream compressedData;
      PointCloudEncoder->encodePointCloud (cloud, compressedData);

      compressedData.read (databuf, 500000);
      int size = compressedData.gcount ();

      shmdata_any_writer_push_data (s_app.writer,
				    databuf, size, myclock, NULL, NULL);
      //clock
      usleep (nsecPeriod / 1000);
      myclock += nsecPeriod;
    }

  return (0);
}

