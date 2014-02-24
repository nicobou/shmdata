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

struct shmdata_base_writer_
{
  GstElement *qserial_;
  GstElement *serializer_;
  GstElement *shmsink_;
  GstElement *parent_bin_;
  gchar *socket_path_;
  gulong client_connected_handler_id_;
  gulong client_disconnected_handler_id_;
  GMutex mutex_;//thread safe
};

//FIXME this should be part of the library
void
shmdata_base_writer_unlink_pad (GstPad * pad)
{
  GstPad *peer;
  if ((peer = gst_pad_get_peer (pad))) {
    if (gst_pad_get_direction (pad) == GST_PAD_SRC)
      gst_pad_unlink (pad, peer);
    else
      gst_pad_unlink (peer, pad);
    //checking if the pad has been requested and releasing it needed 
    GstPadTemplate *pad_templ = gst_pad_get_pad_template (peer);//check if this must be unrefed for GST 1
    if (GST_PAD_TEMPLATE_PRESENCE (pad_templ) == GST_PAD_REQUEST)
      gst_element_release_request_pad (gst_pad_get_parent_element(peer), peer);
    gst_object_unref (peer);
  }
}

//FIXME this should be part of the library
void
shmdata_base_writer_clean_element (GstElement *element)
{
  if (element != NULL )
    if (GST_IS_ELEMENT (element))
      {
	GstIterator *pad_iter;
	pad_iter = gst_element_iterate_pads (element);
	gst_iterator_foreach (pad_iter, (GFunc) shmdata_base_writer_unlink_pad, element);
	gst_iterator_free (pad_iter);
	if (GST_STATE_TARGET (element) != GST_STATE_NULL && GST_STATE (element) != GST_STATE_NULL)
	  if (GST_STATE_CHANGE_ASYNC == gst_element_set_state (element, GST_STATE_NULL))
	    while (GST_STATE (element) != GST_STATE_NULL)
	      gst_element_get_state (element, NULL, NULL, GST_CLOCK_TIME_NONE);//warning this may be blocking
	if (GST_IS_BIN (gst_element_get_parent (element)))
	  gst_bin_remove (GST_BIN (gst_element_get_parent (element)), element);
      }
}

void
shmdata_base_writer_init_members (shmdata_base_writer_t *writer)
{
  writer->qserial_ = NULL;
  writer->serializer_ = NULL;
  writer->shmsink_ = NULL;
  writer->parent_bin_ = NULL;
  writer->socket_path_ = NULL;
  writer->client_connected_handler_id_ = 0;
  writer->client_disconnected_handler_id_ = 0;
  g_mutex_init (&writer->mutex_);
}

void
shmdata_base_writer_close (shmdata_base_writer_t *writer)
{
  if (NULL == writer)
    {
      g_debug("trying to close a NULL writer");
      return;
    }
  g_mutex_lock (&writer->mutex_);
  if (0 != writer->client_connected_handler_id_)
    g_signal_handler_disconnect (writer->shmsink_, 
				 writer->client_connected_handler_id_);
  if (0 != writer->client_disconnected_handler_id_)
    g_signal_handler_disconnect (writer->shmsink_, 
				 writer->client_disconnected_handler_id_);
  shmdata_base_writer_clean_element (writer->qserial_); 
  shmdata_base_writer_clean_element (writer->serializer_); 
  shmdata_base_writer_clean_element (writer->shmsink_); 
  if (NULL != writer->socket_path_)
    {
      g_debug ("closing writer %s", writer->socket_path_);
      g_free (writer->socket_path_);
    }
  else
    g_debug ("closing writer with no socket path");
  g_mutex_unlock (&writer->mutex_);
  g_mutex_clear (&writer->mutex_);
  shmdata_base_writer_init_members (writer);
  g_free (writer);
  writer = NULL;
}

void
shmdata_base_writer_link_branch (shmdata_base_writer_t * writer,
				 GstElement * srcElement)
{
  gst_element_link_many (srcElement,
			 writer->qserial_,
			 writer->serializer_, 
			 writer->shmsink_, 
			 NULL);
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

  GstElement *parent = GST_ELEMENT_PARENT(writer->shmsink_);

  if (GST_STATE (parent) == GST_STATE_NULL && GST_STATE_TARGET (parent) == GST_STATE_NULL)
    return;

  g_debug ("base writer: manual state sync");
  gst_element_set_state (writer->qserial_,GST_STATE_TARGET (parent));
  gst_element_set_state (writer->serializer_,GST_STATE_TARGET (parent));
  gst_element_set_state (writer->shmsink_,GST_STATE_TARGET (parent));
  g_debug ("base writer: manual state sync done");
  return;//parent is already doing something
}

void
shmdata_base_writer_pad_unblocked (GstPad * pad,
				   gboolean blocked, 
				   gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;
  if (blocked)
    g_critical ("Error: pad not in unblocked state");
}

void
shmdata_base_writer_switch_to_new_serializer (GstPad *pad,
					      gboolean blocked,
					      gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;

  g_debug ("base_writer switch_to_new_serializer");
  g_mutex_lock (&context->mutex_);
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
  gst_element_set_state (context->serializer_, GST_STATE_NULL); 
  gst_bin_remove (GST_BIN (bin), context->serializer_);
  //creating and linking the new serializer
  context->serializer_ = gst_element_factory_make ("gdppay", NULL);
  gst_bin_add (bin, context->serializer_);
  GstPad *newSinkPad =
    gst_element_get_static_pad (context->serializer_, "sink");
  GstPad *newSrcPad =
    gst_element_get_static_pad (context->serializer_, "src");
  gst_pad_link (newSrcPad, srcPadPeer);
  gst_pad_link (sinkPadPeer, newSinkPad);
  gst_object_unref (newSinkPad);
  gst_object_unref (newSrcPad);
  gst_object_unref (srcPadPeer);
  gst_object_unref (sinkPadPeer);
  if (!gst_element_set_state (context->serializer_,
			      GST_STATE_TARGET(GST_ELEMENT_PARENT(context->serializer_))))
    g_critical ("Error: issue changing newSerializer state");

  //unblocking data stream
  gst_pad_set_blocked_async (pad,
			     FALSE,
			     (GstPadBlockCallback)
			     (shmdata_base_writer_pad_unblocked),
			     (void *) context);
  g_mutex_unlock (&context->mutex_);

}

void
shmdata_base_writer_on_client_disconnected (GstElement *shmsink,
					    gint num, 
					    gpointer user_data)
{
  g_debug ("client disconnected (number %d)", num);
}

void
shmdata_base_writer_on_client_connected (GstElement *shmsink,
					 gint num, 
					 gpointer user_data)
{
  shmdata_base_writer_t *context = (shmdata_base_writer_t *) user_data;
  if (NULL == context)
    {
      g_debug ("%s: cannot connect to NULL",
	       __FUNCTION__);
      return;
    }
  g_mutex_lock (&context->mutex_);
  if (NULL != context->socket_path_)
    g_debug ("new client connected (socket:%s)", 
	     context->socket_path_);
  GstPad *serializerSinkPad = gst_element_get_static_pad (context->serializer_,
							  "sink");
  GstPad *padToBlock = gst_pad_get_peer (serializerSinkPad);
  gst_object_unref (serializerSinkPad);

  if (!GST_IS_PAD (padToBlock))
    {
      g_warning ("%s: peer pad is not a pad, cannot block");
      return;
    }
  gst_pad_set_blocked_async (padToBlock,
			     TRUE,
			     (GstPadBlockCallback)
			     (shmdata_base_writer_switch_to_new_serializer),
			     (void *) context);
  gst_object_unref (padToBlock);
  g_mutex_unlock (&context->mutex_);
}

void
shmdata_base_writer_make_shm_branch (shmdata_base_writer_t * writer,
				     const char *socketPath)
{
  writer->qserial_ = gst_element_factory_make ("queue", NULL);
  writer->serializer_ = gst_element_factory_make ("gdppay", NULL);
  writer->shmsink_ = gst_element_factory_make ("shmsink", NULL);

  if (!writer->qserial_)
    {
      g_critical ("Writer: \"queue\" element is not available");
      return;
    }
  if (!writer->serializer_)
    {
      g_critical ("Writer: \"gdppay\" element is not available");
      return;
    }
  if (!writer->shmsink_)
    {
      g_critical ("Writer: \"shmsink\" element is not available, consider installing libshmdata");
      return;
    }

  g_object_set (G_OBJECT (writer->shmsink_), "socket-path", socketPath, NULL);
  g_object_set (G_OBJECT (writer->shmsink_), "shm-size", 94967295, NULL);
  g_object_set (G_OBJECT (writer->shmsink_), "sync", FALSE, NULL);
  g_object_set (G_OBJECT (writer->shmsink_), "wait-for-connection", FALSE,
		NULL);

  writer->client_connected_handler_id_ = 
    g_signal_connect (writer->shmsink_, "client-connected",
		      G_CALLBACK (shmdata_base_writer_on_client_connected),
		      writer);
  writer->client_disconnected_handler_id_ = 
    g_signal_connect (writer->shmsink_, "client-disconnected",
		      G_CALLBACK (shmdata_base_writer_on_client_disconnected),
		      writer);

  gst_bin_add_many (GST_BIN (writer->parent_bin_), writer->qserial_,
		    writer->serializer_, writer->shmsink_, NULL);
}



shmdata_base_writer_t *
shmdata_base_writer_new ()
{
  shmdata_base_writer_t *writer =
    (shmdata_base_writer_t *) g_malloc0 (sizeof (shmdata_base_writer_t));
  shmdata_base_writer_init_members (writer);
  return writer;
}

shmdata_base_writer_t *
shmdata_base_writer_init ()
{
  return shmdata_base_writer_new ();
}

gboolean 
shmdata_base_writer_set_path (shmdata_base_writer_t *writer,
			      const char *socket_path)
{
  GFile *shmfile = g_file_new_for_commandline_arg (socket_path);
  if (g_file_query_exists (shmfile, NULL))
    {
      g_object_unref (shmfile);
      g_debug ("file %s exists, could be a writer or a file to delete",
	       socket_path);
      return FALSE; 
    }
  g_object_unref (shmfile);
  g_mutex_lock (&writer->mutex_);
  writer->socket_path_ = g_strdup (socket_path);;
  g_mutex_unlock (&writer->mutex_);
  return TRUE;
}

void
shmdata_base_writer_plug (shmdata_base_writer_t *writer,
			  GstElement *pipeline, 
			  GstElement *srcElement)
{
  if (NULL == writer)
    {
      g_debug ("cannot plug a NULL writer");
      return;
    }
  if (!GST_IS_BIN (pipeline))
    {
      g_warning ("shmdata_base_writer_plug, not a bin");
      return;
    }
  writer->parent_bin_ = pipeline;
  if (NULL == writer->socket_path_) 
    {
      g_warning ("cannot start when socket path has not been set");
      return;
    }
  g_mutex_lock (&writer->mutex_);
  shmdata_base_writer_make_shm_branch (writer, writer->socket_path_);
  shmdata_base_writer_link_branch (writer, srcElement);
  shmdata_base_writer_set_branch_state_as_pipeline (writer);
  g_mutex_unlock (&writer->mutex_);
}

void
shmdata_base_writer_plug_pad (shmdata_base_writer_t *writer,
			      GstElement *pipeline, 
			      GstPad *srcPad)
{
  if (NULL == writer)
    {
      g_debug ("cannot plug a NULL writer");
      return;
    }
  if (!GST_IS_BIN (pipeline))
    {
      g_critical ("shmdata_base_writer_plug, not a bin");
      return;
    }
  writer->parent_bin_ = pipeline; //TODO get rid of the pipeline arg
  if (writer->socket_path_ == NULL) 
    g_critical ("cannot start when socket path has not been set");
  g_mutex_lock (&writer->mutex_);
  shmdata_base_writer_make_shm_branch (writer, writer->socket_path_);
  shmdata_base_writer_link_branch_pad (writer, srcPad);
  shmdata_base_writer_set_branch_state_as_pipeline (writer);
  g_mutex_unlock (&writer->mutex_);
}

