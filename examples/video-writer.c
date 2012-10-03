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

#include <gst/gst.h>
#include <signal.h>
#include "shmdata/base-writer.h"

GstElement *pipeline;
GstElement *tee;
GstElement *qlocalxv;
GstElement *imgsink;
GstElement *timeoverlay;
GstElement *videosource;

const char *socketName;
shmdata_base_writer_t *writer;

//clean up pipeline when ctrl-c
void
leave (int sig)
{
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);

  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));

  exit (sig);
}

//will create the shared video, reading from the tee element
static gboolean
add_shared_video_writer ()
{
  writer = shmdata_base_writer_init ();
  //socketName, pipeline, tee);
  if(shmdata_base_writer_set_path (writer,socketName) == SHMDATA_FILE_EXISTS)
    {
      g_printerr ("**** The file %s exists, therefore a shmdata cannot be operated with this path.\n",socketName);	
      gst_element_set_state (pipeline, GST_STATE_NULL);
      
      g_print ("Deleting pipeline\n");
      gst_object_unref (GST_OBJECT (pipeline));
      exit(0);
    }
  shmdata_base_writer_plug (writer, pipeline, tee);
  g_print ("Now writing to the shared memory\n");
  return FALSE;
}

int
main (int argc, char *argv[])
{
  (void) signal (SIGINT, leave);

  gst_init (&argc, &argv);
  GMainLoop *loop = g_main_loop_new (NULL, FALSE);

  /* Check input arguments */
  if (argc != 2)
    {
      g_printerr ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
    }
  socketName = argv[1];

  /* Create gstreamer elements */
  pipeline = gst_pipeline_new (NULL);

  videosource = gst_element_factory_make ("videotestsrc", NULL);

  timeoverlay = gst_element_factory_make ("timeoverlay", NULL);
  tee = gst_element_factory_make ("tee", NULL);

  qlocalxv = gst_element_factory_make ("queue", NULL); 
  imgsink = gst_element_factory_make ("fakesink", NULL); //("xvimagesink", NULL); 
  

  if (!pipeline || !timeoverlay || !tee  || !qlocalxv || !imgsink )
    {
      g_printerr ("One element could not be created. Exiting.\n");
      return -1;
    }

  /* g_object_set (G_OBJECT (imgsink), "sync", FALSE, NULL); */
  g_object_set (G_OBJECT (videosource), "is-live", TRUE, NULL);

  /*specifying video format */
  GstCaps *videocaps;
  videocaps = gst_caps_new_simple ("video/x-raw-yuv",
				   "format", GST_TYPE_FOURCC,
				   GST_MAKE_FOURCC ('I', '4', '2', '0'),
				   "framerate", GST_TYPE_FRACTION, 60, 1,
				   "pixel-aspect-ratio", GST_TYPE_FRACTION, 1,
				   1, /*"width", G_TYPE_INT, 640, "height",
				   G_TYPE_INT, 480,*/
				   "width", G_TYPE_INT, 1920,
				   "height", G_TYPE_INT, 1080,
				   NULL);

  gst_bin_add_many (GST_BIN (pipeline),
		    videosource, timeoverlay, tee,  qlocalxv, imgsink,  NULL);

  //shared video can be pluged before or after the pipeline state is set to PLAYING
  g_timeout_add (1000, (GSourceFunc) add_shared_video_writer, NULL);
  //or added here
  //add_shared_video_writer();

  /* we link the elements together */
  gst_element_link_filtered (videosource, timeoverlay, videocaps);
  gst_element_link (videosource,timeoverlay);
  gst_element_link (timeoverlay, tee);
  gst_element_link_many (tee, qlocalxv, imgsink, NULL);

  /* Set the pipeline to "playing" state */
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));

  return 0;
}

