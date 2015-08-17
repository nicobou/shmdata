/*
 * Copyright (C) 2015 Nicolas Bouillot (http://www.nicolasbouillot.net)
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
#include "../config.h"
#endif

#include <gst/gst.h>
#include <glib.h>

static int success = 1;  // false 
static GMainLoop *loop = NULL;

// *** gstreamer callbacks
static gboolean bus_call(GstBus *bus,
                         GstMessage *msg,
                         gpointer data){
  GMainLoop *loop =(GMainLoop *) data;
  switch(GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
      g_print("End of stream\n");
      g_main_loop_quit(loop);
      break;
    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;
      gst_message_parse_error(msg, &error, &debug);
      g_free(debug);
      g_printerr("Error: %s\n", error->message);
      g_error_free(error);
      g_main_loop_quit(loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

void on_handoff_cb(GstElement *object, GstBuffer *buf, GstPad *pad, gpointer user_data) {
  success = 0; // true
  g_main_loop_quit(loop);
}

int main () {
  gst_init(NULL, NULL);
#ifdef HAVE_CONFIG_H
  GstRegistry *registry = gst_registry_get();
  gst_registry_scan_path(registry, "./" LT_OBJDIR);
#else
  g_printerr("shmdata plugins not found");
  return -1;
#endif
  loop = g_main_loop_new(NULL, FALSE);
  /* Create gstreamer elements */
  GstElement *pipeline_writer = gst_pipeline_new("audio-writer");
  GstElement *pipeline_reader = gst_pipeline_new("audio-reader");
  GstElement *audiosource = gst_element_factory_make("audiotestsrc", "audiosource");
  GstElement *shmdatasink = gst_element_factory_make("shmdatasink", "shmdata-output");
  GstElement *shmdatasrc = gst_element_factory_make("shmdatasrc", "shmdata-input");
  GstElement *fakesink = gst_element_factory_make("fakesink", "fake");
  if (!pipeline_writer || !pipeline_reader || !audiosource || !shmdatasink || !shmdatasrc || !fakesink) {
    g_printerr("One element could not be created. Exiting.\n"); return -1; }
  GstBus *bus_writer = gst_pipeline_get_bus(GST_PIPELINE(pipeline_writer));
  guint bus_watch_id_writer = gst_bus_add_watch(bus_writer, bus_call, loop);
  gst_object_unref(bus_writer);
  GstBus *bus_reader = gst_pipeline_get_bus(GST_PIPELINE(pipeline_reader));
  guint bus_watch_id_reader = gst_bus_add_watch(bus_reader, bus_call, loop);
  gst_object_unref(bus_reader);
  g_object_set(G_OBJECT(pipeline_writer), "async-handling", TRUE, NULL);
  g_object_set(G_OBJECT(pipeline_reader), "async-handling", TRUE, NULL);
  g_object_set(G_OBJECT(fakesink),
               "silent", TRUE,
               "signal-handoffs", TRUE,
               "sync", FALSE,
               NULL);
  g_signal_connect(G_OBJECT(fakesink), "handoff", (GCallback)on_handoff_cb, NULL);
  g_object_set(G_OBJECT(shmdatasink),
               "socket-path", "/tmp/check-shmdatasrc",
               "sync", FALSE,
               NULL);
  g_object_set(G_OBJECT(shmdatasrc),
               "socket-path", "/tmp/check-shmdatasrc",
               NULL);
  gst_bin_add_many(GST_BIN(pipeline_writer),
                   audiosource, shmdatasink,
                   NULL);
  gst_bin_add_many(GST_BIN(pipeline_reader),
                   shmdatasrc, fakesink,
                   NULL);
  gst_element_link(audiosource, shmdatasink);
  gst_element_link(shmdatasrc, fakesink);
  gst_element_set_state(pipeline_writer, GST_STATE_PLAYING);
  gst_element_set_state(pipeline_reader, GST_STATE_PLAYING);
  g_main_loop_run(loop);
  // cleaning gst
  gst_element_set_state(pipeline_writer, GST_STATE_NULL);
  gst_element_set_state(pipeline_reader, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline_writer));
  gst_object_unref(GST_OBJECT(pipeline_reader));
  g_source_remove(bus_watch_id_writer);
  g_source_remove(bus_watch_id_reader);
  g_main_loop_unref(loop);
  return success;
}
