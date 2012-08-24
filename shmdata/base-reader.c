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
  GstElement *bin_;
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
  //sync_handler
  gboolean install_sync_handler_;
  //state
  gboolean attached_;
  //state boolean
  gboolean initialized_;	//the shared video has been attached once
};

gboolean
shmdata_base_reader_clean_source (gpointer user_data)
{
  shmdata_base_reader_t *context = (shmdata_base_reader_t *) user_data;
  //gst_object_unref (context->sinkPad_);
  if (GST_IS_ELEMENT (context->deserializer_))
      gst_element_set_state (context->deserializer_, GST_STATE_NULL);
  if (GST_IS_ELEMENT (context->source_))
    gst_element_set_state (context->source_, GST_STATE_NULL);
  if (GST_IS_BIN (context->bin_) && GST_IS_ELEMENT (context->deserializer_))
    gst_bin_remove (GST_BIN (context->bin_), context->deserializer_);
  if (GST_IS_BIN (context->bin_) && GST_IS_ELEMENT (context->source_))
    gst_bin_remove (GST_BIN (context->bin_), context->source_);
  return FALSE;
}


void
shmdata_base_reader_attach (shmdata_base_reader_t * reader)
{
  reader->attached_ = TRUE;
  reader->source_ = gst_element_factory_make ("shmsrc", NULL);
  reader->deserializer_ = gst_element_factory_make ("gdpdepay", NULL);
  if (!reader->source_)
    g_critical ("shmsrc element could not be created. \n");
  if (!reader->deserializer_)
    g_critical ("gdpdepay element could not be created. \n");

  g_object_set_data (G_OBJECT (reader->source_), 
		     "shmdata_base_reader",
		     (gpointer)reader);

  g_object_set (G_OBJECT (reader->source_), "socket-path",
		reader->socketName_, NULL);

  gst_bin_add_many (GST_BIN (reader->bin_),
		    reader->source_, reader->deserializer_, NULL);

  reader->deserialPad_ = gst_element_get_static_pad (reader->deserializer_,
						     "src");
  reader->sinkPad_ = gst_element_get_compatible_pad (reader->sink_,
						     reader->deserialPad_,
						     GST_PAD_CAPS
						     (reader->deserialPad_));

  gst_element_link (reader->source_, reader->deserializer_);
  gst_pad_link (reader->deserialPad_, reader->sinkPad_);

  //FIXME sync state with parent
  gst_element_set_state (reader->deserializer_, GST_STATE_PLAYING);
  gst_element_set_state (reader->source_, GST_STATE_PLAYING);

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
      reader->attached_ = FALSE;
      if (GST_IS_ELEMENT (reader->source_) && GST_IS_ELEMENT(reader->deserializer_))
	gst_element_unlink (reader->source_, reader->deserializer_);
      if (GST_IS_PAD(reader->deserialPad_) && GST_IS_PAD(reader->sinkPad_))
	gst_pad_unlink (reader->deserialPad_, reader->sinkPad_);
      //ask for element cleaning in the main thread
      //g_idle_add ((GSourceFunc) shmdata_base_reader_clean_source, reader);
      shmdata_base_reader_clean_source (reader);
    }
}

gboolean
shmdata_base_reader_process_error (shmdata_base_reader_t * reader, GstMessage *msg)
{
  switch (GST_MESSAGE_TYPE (msg))
    {
    case GST_MESSAGE_ERROR:
      {
	gchar *debug;
	GError *error;
	gst_message_parse_error (msg, &error, &debug);
	g_free (debug);
	if (g_strcmp0
	    (GST_ELEMENT_NAME (reader->source_),
	     GST_OBJECT_NAME (msg->src)) == 0)
	  {
	    if (error->code == GST_RESOURCE_ERROR_READ)
	      shmdata_base_reader_detach (reader);
	    g_error_free (error);
	    return TRUE;
	  }
	g_error_free (error);
	break;
      }
    default:
      break;
    }
  return FALSE;
}

GstBusSyncReply
shmdata_base_reader_message_handler (GstBus * bus,
				     GstMessage * msg, gpointer user_data)
{
    
  shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), "shmdata_base_reader");
  if ( reader != NULL)
    {
      if ( shmdata_base_reader_process_error (reader, msg)) 
     	return GST_BUS_DROP; 
      else 
     	return GST_BUS_PASS; 
    }
  return GST_BUS_PASS; 
}

shmdata_base_reader_t *
shmdata_base_reader_new ()
{
  shmdata_base_reader_t *reader =
    (shmdata_base_reader_t *) g_malloc0 (sizeof (shmdata_base_reader_t));
  reader->initialized_ = FALSE;
  reader->on_first_data_ = NULL;
  reader->on_first_data_userData_ = NULL;
  reader->bin_ = NULL;
  reader->install_sync_handler_ = TRUE;
  reader->attached_ = FALSE;
  return reader;
}

void *
shmdata_base_reader_set_callback (shmdata_base_reader_t *reader,
				  shmdata_base_reader_on_first_data cb,
				  void *user_data)
{
  reader->on_first_data_ = cb;
  reader->on_first_data_userData_ = user_data;
}

void *
shmdata_base_reader_install_sync_handler (shmdata_base_reader_t *reader,
					  gboolean install)
{
  reader->install_sync_handler_ = install;
}

gboolean 
shmdata_base_reader_set_bin (shmdata_base_reader_t * reader, GstElement *bin)
{
  if (!GST_IS_BIN (bin))
    {
      g_warning ("base reader: not a bin");
      return FALSE;
    }
  
  reader->bin_ = bin;
  return TRUE;
}

gboolean 
shmdata_base_reader_start (shmdata_base_reader_t * reader, const char *socketPath)
{
  
  
  if (reader->install_sync_handler_)
    {
      g_debug ("installing an async handler");
      //looking for the bus, searching the top level
      GstElement *pipe = reader->bin_;
      
      while (pipe != NULL && !GST_IS_PIPELINE (pipe))
	pipe = GST_ELEMENT_PARENT (pipe);
      
      if( GST_IS_PIPELINE (pipe))
	{
	  GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipe));  
	  
	  gst_bus_set_sync_handler (bus, shmdata_base_reader_message_handler, NULL);  
	  gst_object_unref (bus);  
	}
      else
	{
	  g_warning ("no top level pipeline found when starting, cannot install async_handler");
	  return FALSE;
	}
    }
  
  reader->socketName_ = g_strdup (socketPath);
  
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

  return TRUE;
  
}



shmdata_base_reader_t *
shmdata_base_reader_init (const char *socketName,
			  GstElement *bin,
			  void (*on_first_data) (shmdata_base_reader_t *,
						 void *), void *user_data)
{
  
  shmdata_base_reader_t *reader =
    (shmdata_base_reader_t *) g_malloc0 (sizeof (shmdata_base_reader_t));
  reader->initialized_ = FALSE;
  reader->on_first_data_ = on_first_data;
  reader->on_first_data_userData_ = user_data;
  reader->socketName_ = g_strdup (socketName);
  reader->attached_ = FALSE;
  reader->bin_ = bin;

  //looking for the bus, searching the top level
  GstElement *pipe = reader->bin_;
  
  while (pipe != NULL && !GST_IS_PIPELINE (pipe))
      pipe = GST_ELEMENT_PARENT (pipe);

  if( GST_IS_PIPELINE (pipe))
      {
	  GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipe));  
	  
	  gst_bus_set_sync_handler (bus, shmdata_base_reader_message_handler, NULL);  
	  gst_object_unref (bus);  
      }
  /* else */
  /*     g_debug ("not watching the bus. Application should handle error messages (GST_RESOURCE_ERROR_READ from shmsrc) and call shmdata_base_reader_process_error.\n"); */
  
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
  if (reader != NULL)
    {
      if (reader->socketName_ != NULL)
	g_free (reader->socketName_);
      if (reader->initialized_ && reader->attached_)
	shmdata_base_reader_detach (reader);
      if (reader->shmfile_ != NULL)
	g_object_unref (reader->shmfile_);
      if (reader->dirMonitor_ != NULL)
	g_object_unref (reader->dirMonitor_);
      g_free (reader);
      g_debug ("reader freed %p\n",reader);
    }

}

