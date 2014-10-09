/*
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

GstElement *pipeline = NULL;

/* static gboolean */
/* bus_call (GstBus *bus, */
/*           GstMessage *msg, */
/*           gpointer data) */
/* { */
/*     GMainLoop *loop = (GMainLoop *) data; */

/*     switch (GST_MESSAGE_TYPE (msg)) { */

/*     case GST_MESSAGE_EOS: */
/* 	g_print ("End of stream\n"); */
/* 	g_main_loop_quit (loop); */
/* 	break; */

/*     case GST_MESSAGE_ERROR: { */
/* 	gchar *debug; */
/* 	GError *error; */
      
/* 	gst_message_parse_error (msg, &error, &debug); */
/* 	g_free (debug); */
      
/* 	g_printerr ("Error: %s\n", error->message); */
/* 	g_error_free (error); */
      
/* 	g_main_loop_quit (loop); */
/* 	break; */
/*     } */
/*     default: */
/* 	//g_print ("unknown message type \n"); */
/* 	break; */
/*     } */
/*     return TRUE; */
/* } */

gboolean
check_shm_quit_g_main_loop (gpointer user_data) {
  g_main_loop_quit((GMainLoop *) user_data);
  return FALSE;
}

int
main (int argc,
      char *argv[])
{
    /* Initialisation */
    gst_init (&argc, &argv);
    GMainLoop *loop = g_main_loop_new (NULL, FALSE);

    /* Create gstreamer elements */
    pipeline  = gst_pipeline_new (NULL);

    if (!pipeline) {
	g_printerr ("The pipeline could not be created. Exiting.\n");
	return 1;
    }

    // a writer
    GstElement *fakesrc = gst_element_factory_make ("fakesrc", NULL);
    GstElement *shmsink = gst_element_factory_make ("shmsink", NULL);

    g_object_set (G_OBJECT (shmsink), "socket-path", "/tmp/check_gst_shm", NULL);
    g_object_set (G_OBJECT (shmsink), "shm-size", 94967295, NULL);
    g_object_set (G_OBJECT (shmsink), "sync", FALSE, NULL);
    g_object_set (G_OBJECT (shmsink), "wait-for-connection", FALSE, NULL);

    // a reader
    GstElement *shmsrc = gst_element_factory_make ("shmsrc", NULL);
    GstElement *fakesink = gst_element_factory_make ("fakesink", NULL);
    g_object_set (G_OBJECT (shmsrc),
                  "socket-path", "/tmp/check_gst_shm", NULL);
    
    gst_bin_add_many (GST_BIN (pipeline),
		      fakesrc,
		      shmsink,
                      shmsrc,
                      fakesink,
		      NULL);

    gst_element_link (fakesrc, shmsink);
    gst_element_link (shmsrc, fakesink);
    
    /* message handler */
    /* GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline)); */
    /* gst_bus_add_watch (bus, bus_call, loop); */
    /* gst_object_unref (bus); */

    /* Set the pipeline to "playing" state*/
    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    g_timeout_add(200,
                  check_shm_quit_g_main_loop,         
                  (gpointer)loop);
    
    /* Iterate */
    g_print ("Running...\n");
    g_main_loop_run (loop);
    g_main_loop_unref (loop);

    /* Out of the main loop, clean up nicely */
    g_print ("Returned, stopping playback\n");
    gst_element_set_state (pipeline, GST_STATE_NULL);

    g_print ("Deleting pipeline\n");
    gst_object_unref (pipeline);

    return 0;
}
