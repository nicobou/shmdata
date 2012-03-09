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

#include "shmdata/reader.h"

#ifndef G_LOG_DOMAIN 
#define G_LOG_DOMAIN "shmdata/reader"
#endif

void shmdata_reader_log_handler (const gchar *log_domain,
					 GLogLevelFlags log_level,
					 const gchar *message,
					 gpointer user_data)
{
    g_print ("log handler received: %s, level %d, domain %s, object: %p\n",message,log_level,log_domain,user_data);
}


gboolean shmdata_reader_clean_source (gpointer user_data)
{
    shmdata_reader_t *context = (shmdata_reader_t *)user_data;
    gst_object_unref(context->sinkPad_);
    gst_element_set_state (context->deserializer_, GST_STATE_NULL);
    gst_bin_remove (GST_BIN (context->pipeline_),context->deserializer_);
    gst_element_set_state (context->source_, GST_STATE_NULL);
    gst_bin_remove (GST_BIN (context->pipeline_),context->source_); 
    return FALSE;
}

void shmdata_reader_attach (shmdata_reader_t *reader)
{
    reader->source_       = gst_element_factory_make ("shmsrc",  NULL);
    reader->deserializer_ = gst_element_factory_make ("gdpdepay",  NULL);   
    if ( !reader->source_ || !reader->deserializer_ ) {
	g_critical ("One element could not be created. Exiting.\n");
    }

    g_critical ("tada \n");

    g_object_set (G_OBJECT (reader->source_), "socket-path", reader->socketName_, NULL);
	
    gst_bin_add_many (GST_BIN (reader->pipeline_),
		      reader->source_, 
		      reader->deserializer_, 
		      NULL);

    reader->deserialPad_ = gst_element_get_static_pad (reader->deserializer_,
						       "src");
    reader->sinkPad_ = gst_element_get_compatible_pad (reader->sink_,
						       reader->deserialPad_,
						       GST_PAD_CAPS(reader->deserialPad_));

    gst_element_link (reader->source_, reader->deserializer_);
    gst_pad_link (reader->deserialPad_,reader->sinkPad_);
	
    gst_element_set_state (reader->deserializer_, GST_STATE_PLAYING);
    gst_element_set_state (reader->source_, GST_STATE_PLAYING);
}


void shmdata_reader_file_system_monitor_change (GFileMonitor * monitor,
					   GFile *file,
					   GFile *other_file,
					   GFileMonitorEvent type,
					   gpointer user_data)
{
    char *filename = g_file_get_path (file);
    shmdata_reader_t *context = (shmdata_reader_t *)user_data;
	
    switch (type)
    {
    case G_FILE_MONITOR_EVENT_CREATED:
	if (g_file_equal (file,context->shmfile_)) {
	    if (! context->initialized_)
	    {
		context->initialized_ = TRUE;
		context->on_first_video_data_ (context,context->userData_);
	    }
	    shmdata_reader_attach(context);
	}	  
	break;
	/* case G_FILE_MONITOR_EVENT_DELETED:*/
	/* break; */
	/* case G_FILE_MONITOR_EVENT_CHANGED: */
	/* 	  g_print ("G_FILE_MONITOR_EVENT_CHANGED\n"); */
	/* 	  break; */
	/* case G_FILE_MONITOR_EVENT_ATTRIBUTE_CHANGED: */
	/* 	  g_print ("G_FILE_MONITOR_EVENT_ATTRIBUTE_CHANGED\n"); */
	/* 	  break; */
	/* case G_FILE_MONITOR_EVENT_CHANGES_DONE_HINT: */
	/* 	  g_print ("G_FILE_MONITOR_EVENT_CHANGES_DONE_HINT\n"); */
	/* 	  break; */
	/* case G_FILE_MONITOR_EVENT_PRE_UNMOUNT: */
	/* 	  g_print ("G_FILE_MONITOR_EVENT_PRE_UNMOUNT\n"); */
	/* 	  break; */
	/* case G_FILE_MONITOR_EVENT_UNMOUNTED: */
	/* 	  g_print ("G_FILE_MONITOR_EVENT_UNMOUNTED\n"); */
	/* 	  break; */
    default:
	break;
    }
    g_free (filename);
}



void shmdata_reader_detach (shmdata_reader_t *reader)
{
    gst_element_unlink (reader->source_, reader->deserializer_);
    gst_pad_unlink (reader->deserialPad_,reader->sinkPad_);
    gst_object_unref (reader->deserialPad_);
    gst_element_release_request_pad (reader->sink_,reader->sinkPad_);
    //ask for element cleaning in the main thread
    g_idle_add ((GSourceFunc) shmdata_reader_clean_source,reader);
}

shmdata_reader_t *shmdata_reader_init (const char * socketName, 
				       void(*on_first_video_data)(shmdata_reader_t *, void *),
				       void *user_data) 	
{
    shmdata_reader_t *reader = g_malloc0 (sizeof(shmdata_reader_t));
    reader->initialized_ = FALSE;
    reader->on_first_video_data_ = on_first_video_data;
    reader->userData_ = user_data;
    reader->socketName_ = socketName;
    
    g_log_set_handler (G_LOG_DOMAIN, 
		       (GLogLevelFlags) (G_LOG_LEVEL_MASK |
					 G_LOG_FLAG_FATAL |
					 G_LOG_FLAG_RECURSION), 
		       shmdata_reader_log_handler, 
		       reader);
	
    //monitoring the shared memory file
    reader->shmfile_ = g_file_new_for_commandline_arg (reader->socketName_);
    g_debug("monitoring %s",g_file_get_uri (reader->shmfile_));
    
    if (g_file_query_exists (reader->shmfile_,NULL)){
	reader->initialized_ = TRUE;
	reader->on_first_video_data_ (reader,reader->userData_);
	shmdata_reader_attach (reader);
    }
    
    GFile *dir = g_file_get_parent (reader->shmfile_);
    reader->dirMonitor_ = g_file_monitor_directory (dir, 
						    G_FILE_MONITOR_NONE, 
						    NULL, NULL); 
    g_object_unref(dir);
    g_signal_connect (reader->dirMonitor_, 
		      "changed", 
		      G_CALLBACK (shmdata_reader_file_system_monitor_change), 
		      reader); 
}


GstBusSyncReply shmdata_reader_message_handler (GstBus *bus, 
						GstMessage *msg, 
						gpointer user_data)
{
    switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_ERROR: {
	gchar  *debug;
	GError *error;
	shmdata_reader_t *context = (shmdata_reader_t *)user_data;
	gst_message_parse_error (msg, &error, &debug);
	g_free (debug);
	if (g_strcmp0 (GST_ELEMENT_NAME(context->source_),GST_OBJECT_NAME(msg->src)) == 0 ){
	    if (error->code == GST_RESOURCE_ERROR_READ)
		shmdata_reader_detach (context);
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

void shmdata_reader_set_sink (shmdata_reader_t *reader,
			     GstElement *pipeline, 
			     GstElement *sink)
{
    reader->sink_         = sink;
    reader->pipeline_     = pipeline;

    GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
    gst_bus_set_sync_handler (bus, shmdata_reader_message_handler, reader);
    gst_object_unref (bus);
}
    
gboolean shmdata_reader_close (shmdata_reader_t *reader)
{
    //todo
    g_object_unref(reader->shmfile_);
    g_object_unref(reader->dirMonitor_);
    g_free (reader);
}

    





