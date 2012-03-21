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

#include "shmdata/base-reader.h"

struct shmdata_base_reader_
{
  //pipeline elements
  GstElement *pipeline_;
  GstElement *source_;
  GstElement *deserializer_;
  GstElement *sink_;
  GstPad *sinkPad_;
  GstPad *deserialPad_;
  //monitoring the shm file
  GFile *shmfile_;
  GFileMonitor *dirMonitor_;
  const char *socketName_;
  //user callback
  void (*on_first_data_) (shmdata_base_reader_t *, void *);
  void *on_first_data_userData_;
  //state boolean
  gboolean initialized_;	//the shared video has been attached once
};

gboolean
shmdata_base_reader_clean_source (gpointer user_data)
{
  shmdata_base_reader_t *context = (shmdata_base_reader_t *) user_data;
  //gst_object_unref (context->sinkPad_);
  gst_element_set_state (context->deserializer_, GST_STATE_NULL);
  gst_element_set_state (context->source_, GST_STATE_NULL);
  if (GST_IS_BIN (context->pipeline_))
    gst_bin_remove (GST_BIN (context->pipeline_), context->deserializer_);
  if (GST_IS_BIN (context->pipeline_))
    gst_bin_remove (GST_BIN (context->pipeline_), context->source_);
  return FALSE;
}

void
shmdata_base_reader_attach (shmdata_base_reader_t * reader)
{
  reader->source_ = gst_element_factory_make ("shmsrc", NULL);
  reader->deserializer_ = gst_element_factory_make ("gdpdepay", NULL);
  if (!reader->source_)
    g_critical ("shmsrc element could not be created. Exiting.\n");
  if (!reader->deserializer_)
    g_critical ("gdpdepay element could not be created. Exiting.\n");

  g_object_set (G_OBJECT (reader->source_), "socket-path",
		reader->socketName_, NULL);

  gst_bin_add_many (GST_BIN (reader->pipeline_),
		    reader->source_, reader->deserializer_, NULL);

  reader->deserialPad_ = gst_element_get_static_pad (reader->deserializer_,
						     "src");
  reader->sinkPad_ = gst_element_get_compatible_pad (reader->sink_,
						     reader->deserialPad_,
						     GST_PAD_CAPS
						     (reader->deserialPad_));

  gst_element_link (reader->source_, reader->deserializer_);
  gst_pad_link (reader->deserialPad_, reader->sinkPad_);

  gst_element_set_state (reader->deserializer_, GST_STATE_PLAYING);
  gst_element_set_state (reader->source_, GST_STATE_PLAYING);

  //the following is blocking:
  /* GstState current;    */
  /* gst_element_get_state (reader->pipeline_,&current,NULL,GST_CLOCK_TIME_NONE);    */

  /* if(current != GST_STATE_NULL)    */
  /* {    */
  /*          gst_element_set_state (reader->deserializer_, current);    */
  /*          gst_element_set_state (reader->source_, current);    */
  /* }    */
}

void
shmdata_base_reader_file_system_monitor_change (GFileMonitor * monitor,
						GFile * file,
						GFile * other_file,
						GFileMonitorEvent type,
						gpointer user_data)
{
  char *filename = g_file_get_path (file);
  shmdata_base_reader_t *context = (shmdata_base_reader_t *) user_data;

  switch (type)
    {
    case G_FILE_MONITOR_EVENT_CREATED:
      if (g_file_equal (file, context->shmfile_))
	{
	  if (!context->initialized_)
	    {
	      context->initialized_ = TRUE;
	      context->on_first_data_ (context,
				       context->on_first_data_userData_);
	    }
	  shmdata_base_reader_attach (context);
	}
      break;
      /* case G_FILE_MONITOR_EVENT_DELETED: */
      /* break; */
      /* case G_FILE_MONITOR_EVENT_CHANGED: */
      /*        g_print ("G_FILE_MONITOR_EVENT_CHANGED\n"); */
      /*        break; */
      /* case G_FILE_MONITOR_EVENT_ATTRIBUTE_CHANGED: */
      /*        g_print ("G_FILE_MONITOR_EVENT_ATTRIBUTE_CHANGED\n"); */
      /*        break; */
      /* case G_FILE_MONITOR_EVENT_CHANGES_DONE_HINT: */
      /*        g_print ("G_FILE_MONITOR_EVENT_CHANGES_DONE_HINT\n"); */
      /*        break; */
      /* case G_FILE_MONITOR_EVENT_PRE_UNMOUNT: */
      /*        g_print ("G_FILE_MONITOR_EVENT_PRE_UNMOUNT\n"); */
      /*        break; */
      /* case G_FILE_MONITOR_EVENT_UNMOUNTED: */
      /*        g_print ("G_FILE_MONITOR_EVENT_UNMOUNTED\n"); */
      /*        break; */
    default:
      break;
    }
  g_free (filename);
}

void
shmdata_base_reader_detach (shmdata_base_reader_t * reader)
{
  if (reader != NULL)
    {
      gst_element_unlink (reader->source_, reader->deserializer_);
      if (GST_IS_PAD(reader->deserialPad_) && GST_IS_PAD(reader->sinkPad_))
	gst_pad_unlink (reader->deserialPad_, reader->sinkPad_);
      //ask for element cleaning in the main thread
      g_idle_add ((GSourceFunc) shmdata_base_reader_clean_source, reader);
    }
}

GstBusSyncReply
shmdata_base_reader_message_handler (GstBus * bus,
				     GstMessage * msg, gpointer user_data)
{
  switch (GST_MESSAGE_TYPE (msg))
    {
    case GST_MESSAGE_ERROR:
      {
	gchar *debug;
	GError *error;
	shmdata_base_reader_t *context = (shmdata_base_reader_t *) user_data;
	gst_message_parse_error (msg, &error, &debug);
	g_free (debug);
	if (g_strcmp0
	    (GST_ELEMENT_NAME (context->source_),
	     GST_OBJECT_NAME (msg->src)) == 0)
	  {
	    if (error->code == GST_RESOURCE_ERROR_READ)
	      shmdata_base_reader_detach (context);
	    g_error_free (error);
	    return GST_BUS_DROP;
	  }
	g_error_free (error);
	break;
      }
    default:
      break;
    }
  return GST_BUS_PASS;
}

shmdata_base_reader_t *
shmdata_base_reader_init (const char *socketName,
			  GstElement * pipeline,
			  void (*on_first_data) (shmdata_base_reader_t *,
						 void *), void *user_data)
{

  shmdata_base_reader_t *reader =
    (shmdata_base_reader_t *) g_malloc0 (sizeof (shmdata_base_reader_t));
  reader->initialized_ = FALSE;
  reader->on_first_data_ = on_first_data;
  reader->on_first_data_userData_ = user_data;
  reader->socketName_ = socketName;
  reader->pipeline_ = pipeline;

  GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (reader->pipeline_));
  gst_bus_set_sync_handler (bus, shmdata_base_reader_message_handler, reader);
  gst_object_unref (bus);

  //monitoring the shared memory file
  reader->shmfile_ = g_file_new_for_commandline_arg (reader->socketName_);
  g_debug ("monitoring %s", g_file_get_uri (reader->shmfile_));

  if (g_file_query_exists (reader->shmfile_, NULL))
    {
      reader->initialized_ = TRUE;
      reader->on_first_data_ (reader, reader->on_first_data_userData_);
      shmdata_base_reader_attach (reader);
    }

  GFile *dir = g_file_get_parent (reader->shmfile_);
  reader->dirMonitor_ = g_file_monitor_directory (dir,
						  G_FILE_MONITOR_NONE,
						  NULL, NULL);
  g_object_unref (dir);
  g_signal_connect (reader->dirMonitor_,
		    "changed",
		    G_CALLBACK
		    (shmdata_base_reader_file_system_monitor_change), reader);

  return reader;
}

void
shmdata_base_reader_set_sink (shmdata_base_reader_t * reader,
			      GstElement * sink)
{
  reader->sink_ = sink;
}

void
shmdata_base_reader_close (shmdata_base_reader_t * reader)
{
  shmdata_base_reader_detach (reader);
  g_object_unref (reader->shmfile_);
  g_object_unref (reader->dirMonitor_);
  g_free (reader);
}

