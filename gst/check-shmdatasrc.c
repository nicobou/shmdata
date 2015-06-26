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

static GMainLoop *loop = NULL;

// *** gstreamer callbacks
static gboolean bus_call(GstBus     *bus,
          GstMessage *msg,
          gpointer    data)
{
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

int main () {
  return 0;
  //gstreamer shmdatasink for writing
  GstElement *pipeline,
      *audiosource, *shmdatasink,
      *shmdatasrc, *fakesink;
  GstBus *bus;
  guint bus_watch_id;
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
  pipeline = gst_pipeline_new("audio-player");
  audiosource = gst_element_factory_make("audiotestsrc", "audiosource");
  shmdatasink = gst_element_factory_make("shmdatasink", "shmdata-output");
  shmdatasrc = gst_element_factory_make("shmdatasrc", "shmdata-input");
  fakesink = gst_element_factory_make("fakesink", "fake");
  if (!pipeline || !audiosource || !shmdatasink || !shmdatasrc || !fakesink) {
    g_printerr("One element could not be created. Exiting.\n"); return -1; }
  g_object_set(G_OBJECT(shmdatasink), "socket-path", "/tmp/check-shmdatasrc", NULL);
  g_object_set(G_OBJECT(shmdatasrc), "socket-path", "/tmp/check-shmdatasrc", NULL);
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
  gst_object_unref(bus);
  gst_bin_add_many(GST_BIN(pipeline), audiosource, shmdatasink, shmdatasrc, fakesink, NULL);
  gst_element_link(audiosource, shmdatasink);
  gst_element_link(shmdatasrc, fakesink);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(loop);
  // cleaning gst
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline));
  g_source_remove(bus_watch_id);
  g_main_loop_unref(loop);
  return 0;
}

