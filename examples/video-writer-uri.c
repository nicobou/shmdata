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
#include "shmdata/base-writer.h"

GstElement *pipeline;


const char *socket_name="/tmp/video_writer_uri";
static gchar *uri = "";
static gboolean verbose = FALSE;
static gboolean loop = FALSE;

shmdata_base_writer_t *writer;

static GOptionEntry entries[] =
{
  { "uri", 'u', 0, G_OPTION_ARG_STRING, &uri, "uri to read (ex: file:///tmp/video.mp4)", NULL },
  { "shmdata file", 's', 0, G_OPTION_ARG_STRING, &socket_name, "shmdata file to write (default: /tmp/video_writer_uri)", NULL },
  { "loop", 'l', 0, G_OPTION_ARG_NONE, &verbose, "loop the video", NULL },
  { "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "print messages about what is happening", NULL },
  { NULL }
};


// ------------------------------- cleaning --------------------------------------
void
leave (int sig)
{
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (GST_OBJECT (pipeline));
  exit (sig);
}


//-------------------------------- decoding ------------------------------------------
void uridecodebin_pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data)   
{   
  const gchar *padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));
  
  if (g_str_has_prefix (padname, "video/"))
    {
      g_print ("coucou\n");

      GstElement *queue = gst_element_factory_make ("queue",NULL);
      gst_bin_add (GST_BIN (pipeline), queue);     

      GstPad *queue_sinkpad = gst_element_get_static_pad (queue, "sink");  
      gst_pad_link (pad, queue_sinkpad);     
      gst_object_unref (queue_sinkpad);  
      
      /* GstElement *xvimagesink = gst_element_factory_make ("xvimagesink",NULL); */
      /* gst_bin_add (GST_BIN (pipeline), xvimagesink);      */

      /* gst_element_link (queue,xvimagesink); */

       writer = shmdata_base_writer_init (); 

       if(shmdata_base_writer_set_path (writer,socket_name) == SHMDATA_FILE_EXISTS) 
       	{ 
       	  g_printerr ("**** The file %s exists, therefore a shmdata cannot be operated with this path.\n",socket_name);	 
       	  gst_element_set_state (pipeline, GST_STATE_NULL); 
	  
       	  g_print ("Deleting pipeline\n"); 
       	  gst_object_unref (GST_OBJECT (pipeline)); 
       	  exit(0); 
       	} 
      
       shmdata_base_writer_plug (writer, pipeline, queue); 

      /* g_print ("coucou4\n"); */

      //probing eos        
      //gst_pad_add_event_probe (sample->bin_srcpad, (GCallback) event_probe_cb, (gpointer)sample);        

      if (!gst_element_sync_state_with_parent (queue))        
       	g_error ("pb syncing queue with pipeline\n"); 

      /* if (!gst_element_sync_state_with_parent (xvimagesink))         */
      /*  	g_error ("pb syncing xv with pipeline\n");  */

    }
  else
    {
      g_print ("ignoring not handled data type: %s\n",padname);
      GstElement *fake = gst_element_factory_make ("fakesink", NULL);
      gst_bin_add (GST_BIN (pipeline),fake);
      if (!gst_element_sync_state_with_parent (fake))      
	g_error ("pb syncing datastream state: %s\n",padname);
      GstPad *fakepad = gst_element_get_static_pad (fake,"sink");
      gst_pad_link (pad,fakepad);
      gst_object_unref (fakepad);
    }
  return;   
}   



//---------------------------- main ---------------------------------------------------

int
main (int argc, char *argv[])
{

  //command line options
  GError *error = NULL;
  GOptionContext *context;
  context = g_option_context_new ("- write a single video stream to a shmdata");
  g_option_context_add_main_entries (context, entries, NULL);
  if (!g_option_context_parse (context, &argc, &argv, &error))
    {
      g_print ("option parsing failed: %s\n", error->message);
      exit (1);
    } 
  
  GMainLoop *loop;

  (void) signal (SIGINT, leave);

  /* always init first */
  gst_init (&argc, &argv);

  /* the pipeline to hold everything */
  pipeline = gst_pipeline_new (NULL);
  g_assert (pipeline);


  /* we need to run a GLib main loop to get the messages */
  loop = g_main_loop_new (NULL, FALSE);
  
  GstElement *src = gst_element_factory_make ("uridecodebin",NULL);   
  g_signal_connect (G_OBJECT (src), 
		    "pad-added", 
		    (GCallback) uridecodebin_pad_added_cb , 
		    NULL);
  g_object_set (G_OBJECT (src),  
   		"ring-buffer-max-size",(guint64)200000000, 
   		"download",TRUE, 
   		"use-buffering",TRUE, 
   		"async-handling",TRUE, 
   		"buffer-duration",9223372036854775807, 
   		NULL); 
  g_object_set (G_OBJECT (src), "uri",   
    		uri, NULL);  
  

  gst_bin_add (GST_BIN (pipeline), src);

  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  g_main_loop_run (loop);

  if (verbose)
    g_print ("stopping sender\n");

  gst_element_set_state (pipeline, GST_STATE_NULL);

  return 0;
}

