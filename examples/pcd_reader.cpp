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

static gchar *socket_path = "/tmp/pcd_to_read";
static gboolean verbose = FALSE;

static GOptionEntry entries[] =
{
  
  { "socket-path", 's', 0, G_OPTION_ARG_STRING, &socket_path, "socket path for writing (default /tmp/pcd_to_read)", NULL },
  { "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "print messages about what is happening", NULL },
  { NULL, NULL, 0, G_OPTION_ARG_NONE, NULL, NULL, NULL }
};


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
  s_app.on = false;
}

int
main (int argc, char *argv[])
{

  //command line options
  GError *error = NULL;
  GOptionContext *context;
  context = g_option_context_new ("- read a compressed pcd frames from a shmdata-any, decompress and display.");
  g_option_context_add_main_entries (context, entries, NULL);
  if (!g_option_context_parse (context, &argc, &argv, &error))
    {
      g_print ("option parsing failed: %s\n", error->message);
      exit (1);
    } 
  
  //close when ctrl-c
  (void) signal (SIGINT, leave);
  
  s_app.PointCloudDecoder =
    new pcl::octree::PointCloudCompression < pcl::PointXYZRGB > ();
  s_app.viewer =
    new pcl::visualization::CloudViewer ("Compressed Point Cloud receiver");

    
  s_app.reader = shmdata_any_reader_init ();
  if (verbose)
    shmdata_any_reader_set_debug (s_app.reader, SHMDATA_ENABLE_DEBUG);
  else
    shmdata_any_reader_set_debug (s_app.reader, SHMDATA_DISABLE_DEBUG);
  shmdata_any_reader_set_on_data_handler (s_app.reader, &on_data, NULL);
  shmdata_any_reader_set_data_type (s_app.reader, "application/x-pcd");
  shmdata_any_reader_start (s_app.reader, socket_path);

  //shmdata_any_reader is non blocking so waiting for ctrl-c to quit
  s_app.on = true;
  while (s_app.on)
    {
      sleep (1);
    }

  shmdata_any_reader_close (s_app.reader);
  delete s_app.PointCloudDecoder;
  delete s_app.viewer;
  return 0;
}

