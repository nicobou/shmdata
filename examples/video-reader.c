/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <signal.h>
#include "shmdata/base-reader.h"

GstElement *pipeline;
GstElement *shmDisplay;
GstElement *ffmpegcolorspace;
GstElement *funnel;

const char *socketName;
shmdata_base_reader_t *reader;

static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;

  switch (GST_MESSAGE_TYPE (msg))
    {

    case GST_MESSAGE_EOS:
      g_print ("message %s from %s\n",GST_MESSAGE_TYPE_NAME(msg),GST_MESSAGE_SRC_NAME(msg));
      gst_element_set_state (pipeline, GST_STATE_NULL);
      g_main_loop_quit (loop);
      break;

    case GST_MESSAGE_ERROR:
      {
	gchar *debug;
	GError *error;

	gst_message_parse_error (msg, &error, &debug);
	g_free (debug);

	g_printerr ("Error: %s (message %s from %s)\n", error->message,GST_MESSAGE_TYPE_NAME(msg),GST_MESSAGE_SRC_NAME(msg));
	g_error_free (error);

	g_print ("Now nulling: \n");
	gst_element_set_state (pipeline, GST_STATE_NULL);
	g_main_loop_quit (loop);
	break;
      }
    case GST_MESSAGE_STATE_CHANGED:
      {
	/* GstState old_state, new_state; */
	/* gst_message_parse_state_changed (msg, &old_state, &new_state, NULL); */
	/* g_print ("Element %s changed state from %s to %s.\n", */
	/* 	 GST_OBJECT_NAME (msg->src), */
	/* 	 gst_element_state_get_name (old_state), */
	/* 	 gst_element_state_get_name (new_state)); */
	break;
      }
    default:
      {
	//g_print ("message %s from %s\n",GST_MESSAGE_TYPE_NAME(msg),GST_MESSAGE_SRC_NAME(msg));
	break;
      }
    }

  return TRUE;
}

void
leave (int sig)
{
  
  g_print ("Returned, closing the reader\n");
  shmdata_base_reader_close (reader);

  exit (sig);
}

void
on_first_video_data (shmdata_base_reader_t * context, void *user_data)
{
  g_print ("creating element to display the shared video \n");

#ifdef HAVE_OSX
  shmDisplay = gst_element_factory_make ("osxvideosink", NULL);
#else
  shmDisplay = gst_element_factory_make ("xvimagesink", NULL);
#endif

  ffmpegcolorspace = gst_element_factory_make ("ffmpegcolorspace", NULL);
  //in order to be dynamic, the shared video is linking to an
  //element accepting request pad (as funnel of videomixer)
  funnel = gst_element_factory_make ("funnel", NULL);
  g_object_set (G_OBJECT (shmDisplay), "sync", FALSE, NULL);

  if (!shmDisplay || !funnel || !ffmpegcolorspace)
    {
      g_printerr ("One element could not be created. \n");
    }

  //element must have the same state as the pipeline
  gst_bin_add_many (GST_BIN (pipeline), funnel, ffmpegcolorspace, shmDisplay, NULL);
  gst_element_link_many (funnel, ffmpegcolorspace, shmDisplay, NULL);

  //now tells the shared video reader where to write the data
  shmdata_base_reader_set_sink (context, funnel);

  gst_element_set_state (shmDisplay, GST_STATE_PLAYING);
  gst_element_set_state (ffmpegcolorspace, GST_STATE_PLAYING);
  gst_element_set_state (funnel, GST_STATE_PLAYING);

}

static gboolean
add_shared_video_reader (gpointer user_data)
{
  GstElement *pipeline = (GstElement *) user_data;
  g_print ("add shared video reader\n");
  reader = shmdata_base_reader_new ();
  shmdata_base_reader_set_callback (reader, &on_first_video_data, NULL);
  shmdata_base_reader_set_bin (reader, pipeline);
  shmdata_base_reader_install_sync_handler (reader, TRUE);
  shmdata_base_reader_start (reader, socketName);
  //    shmdata_base_reader_init (socketName, pipeline, &on_first_video_data,
  //NULL);
  return FALSE;
}

void
my_log_handler (const gchar * log_domain,
		GLogLevelFlags log_level,
		const gchar * message, gpointer user_data)
{
  g_print ("%s\n", message);
}

int
main (int argc, char *argv[])
{

  (void) signal (SIGINT, leave);

  GMainLoop *loop;
  GstBus *bus;

  if (argc != 2)
    {
      g_printerr ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
    }
  socketName = argv[1];

  /* Initialisation */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

#ifdef HAVE_CONFIG_H 
  GstRegistry *registry; 
  registry = gst_registry_get_default(); 
  gst_registry_scan_path (registry, SHMDATA_SHM_GST_PLUGIN_BUILD_PATH); 
  gst_registry_scan_path (registry, SHMDATA_GST_PLUGIN_PATH);
#endif 
  
  //get logs
  g_print ("set logs\n");
  g_log_set_default_handler (my_log_handler, NULL);

  /* Create gstreamer elements */
  pipeline = gst_pipeline_new (NULL);
  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* GstElement *localVideoSource = gst_element_factory_make ("videotestsrc", NULL); */
  /* GstElement *localDisplay = gst_element_factory_make ("xvimagesink", NULL); */

  if (!pipeline /* || !localVideoSource || !localDisplay */ )
    {
      g_printerr ("One element could not be created. Exiting.\n");
      return -1;
    }

  /* gst_bin_add_many (GST_BIN (pipeline), localVideoSource, localDisplay, NULL); */
  /* gst_element_link (localVideoSource, localDisplay); */

  // shmdata_base_reader_init (socketName,&on_first_video_data);
  g_timeout_add (1000, (GSourceFunc) add_shared_video_reader, pipeline);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Iterate */
  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);

  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));

  return 0;
}

