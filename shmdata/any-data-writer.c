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

#include "shmdata/base-writer.h"
#include "shmdata/any-data-writer.h"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#include <gst/app/gstappsink.h>

struct shmdata_any_writer_
{
  shmdata_base_writer_t *base_writer_;
  //debug
  int debug_;
  //data type
  gchar *type_;
  GstCaps *data_caps_;
  //gstreamer
  GstElement *pipeline_;
  GstElement *src_;
  GstElement *id_;
};

void
shmdata_any_writer_log_handler (const gchar * log_domain,
				GLogLevelFlags log_level,
				const gchar * message, gpointer user_data)
{
  if (g_strcmp0 (log_domain, G_LOG_DOMAIN) == 0)
    {
      shmdata_any_writer_t *context = (shmdata_any_writer_t *) user_data;
      switch (log_level)
	{
	case G_LOG_LEVEL_ERROR:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s, ERROR: %s\n", G_LOG_DOMAIN, message);
	  break;
	case G_LOG_LEVEL_CRITICAL:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s, CRITICAL: %s\n", G_LOG_DOMAIN, message);
	  break;
	case G_LOG_LEVEL_WARNING:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s, WARNING: %s\n", G_LOG_DOMAIN, message);
	  break;
	case G_LOG_LEVEL_MESSAGE:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s, MESSAGE: %s\n", G_LOG_DOMAIN, message);
	  break;
	case G_LOG_LEVEL_INFO:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s, INFO: %s\n", G_LOG_DOMAIN, message);
	  break;
	case G_LOG_LEVEL_DEBUG:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s, DEBUG: %s\n", G_LOG_DOMAIN, message);
	  break;
	default:
	  if (context->debug_ == SHMDATA_ENABLE_DEBUG)
	    g_print ("%s: %s\n", G_LOG_DOMAIN, message);
	  break;
	}
    }
}

shmdata_any_writer_t *
shmdata_any_writer_init ()
{
  shmdata_any_writer_t *writer =
    (shmdata_any_writer_t *) g_malloc0 (sizeof (shmdata_any_writer_t));
  writer->debug_ = SHMDATA_DISABLE_DEBUG;
  g_log_set_default_handler (shmdata_any_writer_log_handler, writer);
  writer->type_ = NULL;
  writer->data_caps_ = NULL;

  gst_init (NULL, NULL);

  writer->pipeline_ = gst_pipeline_new (NULL);
  g_assert (writer->pipeline_);

  writer->base_writer_ =
    shmdata_base_writer_init ();

  return writer;
}

void
shmdata_any_writer_set_debug (shmdata_any_writer_t * context, int debug)
{
  context->debug_ = debug;
}

int
shmdata_any_writer_set_path (shmdata_any_writer_t * writer,
				 const char *socketPath)
{
  return (int)shmdata_base_writer_set_path (writer->base_writer_,
					 socketPath);
}


void
shmdata_any_writer_start (shmdata_any_writer_t * writer)
{

  writer->src_ = gst_element_factory_make ("appsrc", NULL);
  g_assert (writer->src_);
  gst_bin_add (GST_BIN (writer->pipeline_), writer->src_);

  if (writer->data_caps_ == NULL)
    writer->data_caps_ = gst_caps_new_simple ("application/shmdata_", NULL);
  gst_app_src_set_caps (GST_APP_SRC (writer->src_), writer->data_caps_);

  writer->id_ = gst_element_factory_make ("identity", NULL);
  g_assert (writer->id_);
  gst_bin_add (GST_BIN (writer->pipeline_), writer->id_);

  shmdata_base_writer_plug (writer->base_writer_, writer->pipeline_, writer->id_);
  gst_element_link (writer->src_, writer->id_);

  gst_element_set_state (writer->pipeline_, GST_STATE_PLAYING);
}

void
shmdata_any_writer_set_data_type (shmdata_any_writer_t * writer,
				  const char *type)
{
  writer->type_ = g_strdup (type);
  writer->data_caps_ = gst_caps_from_string (writer->type_);
}

void
shmdata_any_writer_push_data (shmdata_any_writer_t * context,
			      void *data,
			      int size,
			      unsigned long long timestamp,
			      void (*done_with_data) (void *),
			      void *user_data)
{
  GstBuffer *buf;
  buf = gst_app_buffer_new (data, size, done_with_data, user_data);
  GST_BUFFER_TIMESTAMP (buf) = (GstClockTime) (timestamp);
  gst_app_src_push_buffer (GST_APP_SRC (context->src_), buf);
}

void
shmdata_any_writer_push_data_with_duration (shmdata_any_writer_t * context,
					    void *data,
					    int size,
					    unsigned long long timestamp,
					    unsigned long long duration,
					    unsigned long long offset,
					    unsigned long long offset_end,
					    void (*done_with_data) (void *),
					    void *user_data)
{
  GstBuffer *buf;
  buf = gst_app_buffer_new (data, size, done_with_data, user_data);
  GST_BUFFER_TIMESTAMP (buf) = (GstClockTime) (timestamp);
  GST_BUFFER_DURATION (buf) = (GstClockTime) (duration);
  GST_BUFFER_OFFSET (buf) = (GstClockTime) (offset);
  GST_BUFFER_OFFSET_END (buf) = (GstClockTime) (offset_end);
  gst_app_src_push_buffer (GST_APP_SRC (context->src_), buf);
}

void
shmdata_any_writer_close (shmdata_any_writer_t * writer)
{
  /* push EOS */
  //gst_app_src_end_of_stream (GST_APP_SRC (app->src));
  if (writer != NULL)
    {
      if (writer->base_writer_ != NULL)
	shmdata_base_writer_close (writer->base_writer_);
      if (writer->pipeline_ != NULL)
	{
	  gst_element_set_state (writer->pipeline_, GST_STATE_NULL);
	  gst_object_unref (GST_OBJECT (writer->pipeline_));
	}
      if (writer->data_caps_ != NULL)
	gst_caps_unref (writer->data_caps_);
      if (writer->type_ != NULL)
	g_free (writer->type_);
      g_free (writer);
    }
}

