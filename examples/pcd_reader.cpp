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
#include "shmdata/any-data-reader.h"

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

typedef struct _App App;
struct _App
{
  shmdata_any_reader_t *reader;
    pcl::visualization::CloudViewer * viewer;
    pcl::octree::PointCloudCompression < pcl::PointXYZRGB >
    *PointCloudDecoder;
  bool on;
};

App s_app;

void
on_data (shmdata_any_reader_t * reader,
	 void *shmbuf,
	 void *data,
	 int data_size,
	 unsigned long long timestamp,
	 const char *type_description, void *user_data)
{

  pcl::PointCloud < pcl::PointXYZRGB >::Ptr cloudOut (new pcl::PointCloud <
						      pcl::PointXYZRGB > ());
  // stringstream to store compressed point cloud
  std::stringstream compressedData;
  compressedData.write ((const char *) data, data_size);
  //decode and show the point cloud
  s_app.PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
  s_app.viewer->showCloud (cloudOut);

  shmdata_any_reader_free (shmbuf);
}

void
leave (int sig)
{
  shmdata_any_reader_close (s_app.reader);
  delete s_app.PointCloudDecoder;
  delete s_app.viewer;
  s_app.on = false;
  //exit(sig);
}

int
main (int argc, char *argv[])
{
  (void) signal (SIGINT, leave);

  s_app.PointCloudDecoder =
    new pcl::octree::PointCloudCompression < pcl::PointXYZRGB > ();
  s_app.viewer =
    new pcl::visualization::CloudViewer ("Compressed Point Cloud receiver");

  if (argc != 2)
    {
      g_printerr ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
    }

  s_app.reader = shmdata_any_reader_init ();
  shmdata_any_reader_set_debug (s_app.reader, SHMDATA_ENABLE_DEBUG);
  shmdata_any_reader_set_on_data_handler (s_app.reader, &on_data, NULL);
  shmdata_any_reader_set_data_type (s_app.reader, "application/x-pcd");
  shmdata_any_reader_start (s_app.reader, argv[1]);

  //shmdata_any_reader is non blocking
  s_app.on = true;
  while (s_app.on)
    {
      sleep (1);
    }

  return 0;
}

