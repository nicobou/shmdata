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

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <gst/gst.h>
#include <glib.h>
#include "shmdata/clogger.h"
#include "shmdata/cfollower.h"

static GMainLoop *loop = NULL;
static int server_interactions = 0;
static res = 1;  // fail by default

// *** follower callbacks
void mylog(void *user_data, const char *str) {
  printf("%s\n", str);
}

void on_server_connect(void *user_data, const char *type_descr) {
  printf("server connect: type %s\n", type_descr);
  ++server_interactions;
}

void on_server_disconnect(void *user_data) {
  printf("server dicsonnect");
  ++server_interactions;
}

void on_data(void *user_data, void *data, size_t size) {
  printf("new data for client: size %zu ptr %p\n", size, data);
  res = 0;
  g_main_loop_quit(loop);
}

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

// *** main
int main() {
  //makeing a follower
  ShmdataLogger logger = shmdata_make_logger(&mylog, 
                                             &mylog, 
                                             &mylog, 
                                             &mylog, 
                                             &mylog, 
                                             &mylog, 
                                             NULL); 
  assert(NULL != logger); 
  ShmdataFollower follower = shmdata_make_follower("/tmp/check-shmdatasink", 
                                                   &on_data, 
                                                   &on_server_connect, 
                                                   &on_server_disconnect, 
                                                   NULL, 
                                                   logger); 
  assert(NULL != follower); 
  //gstreamer shmdatasink for writing
  GstElement *pipeline, *audiosource, *shmdatasink;
  GstBus *bus;
  guint bus_watch_id;
  gst_init(NULL, NULL);
#ifdef HAVE_CONFIG_H
  GstRegistry *registry = gst_registry_get();
  gst_registry_scan_path(registry, "./" LT_OBJDIR);
#else
  g_printerr("shmdatasink plugin not found");
return -1;
#endif
  loop = g_main_loop_new(NULL, FALSE);
  /* Create gstreamer elements */
  pipeline = gst_pipeline_new("audio-player");
  audiosource = gst_element_factory_make("audiotestsrc", "audiosource");
  shmdatasink = gst_element_factory_make("shmdatasink", "shmdata-output");
  if (!pipeline || !audiosource || !shmdatasink) {
    g_printerr("One element could not be created. Exiting.\n"); return -1; }
  g_object_set(G_OBJECT(shmdatasink), "socket-path", "/tmp/check-shmdatasink", NULL);
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
  gst_object_unref(bus);
  gst_bin_add_many(GST_BIN(pipeline), audiosource, shmdatasink, NULL);
  gst_element_link(audiosource, shmdatasink);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(loop);
  // cleaning gst
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline));
  g_source_remove(bus_watch_id);
  g_main_loop_unref(loop);
  // cleaning follower 
  shmdata_delete_follower(follower); 
  shmdata_delete_logger(logger); 
  return res;
}

