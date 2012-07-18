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

#include "shmdata/base-writer.h"

struct shmdata_base_writer_
{
  GstElement *qserial_;
  GstElement *serializer_;
  GstElement *shmsink_;
  GstElement *pipeline_;
  gchar *socket_path_;
  gboolean timereset_;
  GstClockTime timeshift_;
};

void
shmdata_base_writer_close (shmdata_base_writer_t * writer)
{
  if (writer->socket_path_ != NULL)
    g_free (writer->socket_path_);
  if (writer != NULL)
    g_free (writer);
}

void
shmdata_base_writer_link_branch (shmdata_base_writer_t * writer,
				 GstElement * srcElement)
{
  gst_element_link_many (srcElement,
			 writer->qserial_,
			 writer->serializer_, writer->shmsink_, NULL);
}

void
shmdata_base_writer_link_branch_pad (shmdata_base_writer_t * writer,
				     GstPad * srcPad)
{
  GstPad *sinkPad = gst_element_get_static_pad (writer->qserial_, "sink");
  /* if (sinkPad) */
  /*   g_warning ("sinkPad"); */
  /* GstPadLinkReturn lres =  */ 
  gst_pad_link (srcPad, sinkPad);
  /* if (lres == GST_PAD_LINK_OK) */
  /*   g_critical ("lres == GST_PAD_LINK_OK"); */
  gst_object_unref (sinkPad);
  gst_element_link_many (writer->qserial_,
			 writer->serializer_, writer->shmsink_, NULL);
}

void
shmdata_base_writer_set_branch_state_as_pipeline (shmdata_base_writer_t *
						  writer)
{
  GstState current;
  gst_element_get_state (writer->pipeline_, &current, NULL,
			 GST_CLOCK_TIME_NONE);

  if (current != GST_STATE_NULL)
    {
      gst_element_set_state (writer->qserial_, current);
      gst_element_set_state (writer->serializer_, current);
      gst_element_set_state (writer->shmsink_, current);
    }
}

gboolean
shmdata_base_writer_reset_time (GstPad * pad,
				GstMiniObject * mini_obj, gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;
  if (GST_IS_EVENT (mini_obj))
    {
    }
  else if (GST_IS_BUFFER (mini_obj))
    {
      GstBuffer *buffer = GST_BUFFER_CAST (mini_obj);
      if (context->timereset_)
	{
	  context->timeshift_ = GST_BUFFER_TIMESTAMP (buffer);
	  context->timereset_ = FALSE;
	}
      GST_BUFFER_TIMESTAMP (buffer) =
	GST_BUFFER_TIMESTAMP (buffer) - context->timeshift_;
    }
  else if (GST_IS_MESSAGE (mini_obj))
    {
    }

  return TRUE;
}

void
shmdata_base_writer_pad_unblocked (GstPad * pad,
				   gboolean blocked, gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;
  if (blocked)
    g_critical ("Error: pad not in unblocked state");
  else
    context->timereset_ = TRUE;
}

void
shmdata_base_writer_switch_to_new_serializer (GstPad * pad,
					      gboolean blocked,
					      gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;

  //unlink the old serializer
  GstPad *srcPad = gst_element_get_static_pad (context->serializer_, "src");
  GstPad *srcPadPeer = gst_pad_get_peer (srcPad);
  if (!gst_pad_unlink (srcPad, srcPadPeer))
    g_critical ("Error: cannot unlink src");

  GstPad *sinkPad = gst_element_get_static_pad (context->serializer_, "sink");
  GstPad *sinkPadPeer = gst_pad_get_peer (sinkPad);
  if (!gst_pad_unlink (sinkPadPeer, sinkPad))
    g_critical ("Error: cannot unlink sink");

  gst_object_unref (srcPad);
  gst_object_unref (sinkPad);

  GstBin *bin = GST_BIN (GST_ELEMENT_PARENT (context->serializer_));

  // supposed to be PLAYING, possible issue because of not taking care
  // of pending state
  GstState current;
  gst_element_get_state (context->serializer_, &current, NULL,
			 GST_CLOCK_TIME_NONE);

  //get rid of the old serializer TODO ensure object has been cleaned up
  gst_element_set_state (context->serializer_, GST_STATE_NULL);
  //waiting for possible async state change
  gst_element_get_state (context->serializer_, NULL, NULL,
			 GST_CLOCK_TIME_NONE);

  //creating and linking the new serializer
  context->serializer_ = gst_element_factory_make ("gdppay", NULL);
  if (gst_element_set_state (context->serializer_, current) !=
      GST_STATE_CHANGE_SUCCESS)
    g_critical ("Error: issue changing newSerializer state");
  else
    {
      gst_bin_add (bin, context->serializer_);
      GstPad *newSinkPad =
	gst_element_get_static_pad (context->serializer_, "sink");
      GstPad *newSrcPad =
	gst_element_get_static_pad (context->serializer_, "src");
      gst_pad_link (newSrcPad, srcPadPeer);
      gst_pad_link (sinkPadPeer, newSinkPad);
      gst_object_unref (newSinkPad);
      gst_object_unref (newSrcPad);
    }
  gst_object_unref (srcPadPeer);
  gst_object_unref (sinkPadPeer);

  //unblocking data stream
  gst_pad_set_blocked_async (pad,
			     FALSE,
			     (GstPadBlockCallback)
			     (shmdata_base_writer_pad_unblocked),
			     (void *) context);
}

void
shmdata_base_writer_on_client_disconnected (GstElement * shmsink,
					    gint num, gpointer user_data)
{
  g_message ("client disconnected (number %d)", num);
}

void
shmdata_base_writer_on_client_connected (GstElement * shmsink,
					 gint num, gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;

  g_message ("new client connected (number %d)", num);

  GstPad *serializerSinkPad =
    gst_element_get_static_pad (context->serializer_, "sink");
  GstPad *padToBlock = gst_pad_get_peer (serializerSinkPad);

  if (!gst_pad_set_blocked_async (padToBlock,
				  TRUE,
				  (GstPadBlockCallback)
				  (shmdata_base_writer_switch_to_new_serializer),
				  (void *) context))
    g_critical ("Error: when requesting the pad to be blocked");
  gst_object_unref (serializerSinkPad);
  gst_object_unref (padToBlock);
}

void
shmdata_base_writer_make_shm_branch (shmdata_base_writer_t * writer,
				     const char *socketPath)
{
  writer->qserial_ = gst_element_factory_make ("queue", NULL);
  writer->serializer_ = gst_element_factory_make ("gdppay", NULL);
  writer->shmsink_ = gst_element_factory_make ("shmsink", NULL);

  if (!writer->qserial_ || !writer->serializer_ || !writer->shmsink_)
    {
      g_critical ("Writer: One gstreamer element could not be created.");
    }

  g_object_set (G_OBJECT (writer->shmsink_), "socket-path", socketPath, NULL);
  g_object_set (G_OBJECT (writer->shmsink_), "shm-size", 94967295, NULL);
  g_object_set (G_OBJECT (writer->shmsink_), "sync", FALSE, NULL);
  g_object_set (G_OBJECT (writer->shmsink_), "wait-for-connection", FALSE,
		NULL);

  //adding a probe for reseting timestamp when reconnecting
  GstPad *qserialPad = gst_element_get_pad (writer->qserial_, "src");
  gst_pad_add_data_probe (qserialPad,
			  G_CALLBACK (shmdata_base_writer_reset_time),
			  writer);
  gst_object_unref (qserialPad);

  g_signal_connect (writer->shmsink_, "client-connected",
		    G_CALLBACK (shmdata_base_writer_on_client_connected),
		    writer);

  g_signal_connect (writer->shmsink_, "client-disconnected",
		    G_CALLBACK (shmdata_base_writer_on_client_disconnected),
		    writer);

  gst_bin_add_many (GST_BIN (writer->pipeline_), writer->qserial_,
		    writer->serializer_, writer->shmsink_, NULL);
}

shmdata_base_writer_t *
shmdata_base_writer_init ()
{
  shmdata_base_writer_t *writer =
    (shmdata_base_writer_t *) g_malloc0 (sizeof (shmdata_base_writer_t));
  writer->timereset_ = FALSE;
  writer->timeshift_ = 0;

  return writer;
}

gboolean 
shmdata_base_writer_set_path (shmdata_base_writer_t * writer,
			      const char *socket_path)
{
  GFile *shmfile = g_file_new_for_commandline_arg (socket_path);
  if (g_file_query_exists (shmfile, NULL))
    {
      g_object_unref (shmfile);
      g_critical ("file %s exists, could be a writer or a file to delete",
		  socket_path);
      g_object_unref (shmfile);
      return FALSE; 
    }
    g_object_unref (shmfile);

    writer->socket_path_ = g_strdup (socket_path);;
  
  return TRUE;
}

void
shmdata_base_writer_plug (shmdata_base_writer_t *writer,
			  GstElement *pipeline, 
			  GstElement *srcElement)
{
  writer->pipeline_ = pipeline;
  if (writer->socket_path_ == NULL) 
        g_critical ("cannot start when socket path has not been set");
  shmdata_base_writer_make_shm_branch (writer, writer->socket_path_);
  shmdata_base_writer_link_branch (writer, srcElement);
  shmdata_base_writer_set_branch_state_as_pipeline (writer);
}

void
shmdata_base_writer_plug_pad (shmdata_base_writer_t * writer,
			      GstElement * pipeline, 
			      GstPad * srcPad)
{
  writer->pipeline_ = pipeline; //TODO get rid of the pipeline arg
  if (writer->socket_path_ == NULL) 
    g_critical ("cannot start when socket path has not been set");
  shmdata_base_writer_make_shm_branch (writer, writer->socket_path_);
  shmdata_base_writer_link_branch_pad (writer, srcPad);
  shmdata_base_writer_set_branch_state_as_pipeline (writer);
}

