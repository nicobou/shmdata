#include "shmdata.h"

namespace shmdata 
{ 
    Reader::Reader ()
    {}

    Reader::Reader (const std::string socketName, void(*on_first_video_data)(Reader *, void *),void *user_data) 	
    {
	initialized_ = FALSE;
	on_first_video_data_ = on_first_video_data;
	userData_ = user_data;
	socketName_.append(socketName);

        //monitoring the shared memory file
	shmfile_ = g_file_new_for_commandline_arg (socketName_.c_str());
	if (shmfile_ == NULL) {
	    g_printerr ("argument not valid. \n");
	}
	
	if (g_file_query_exists (shmfile_,NULL)){
	    initialized_ = TRUE;
	    on_first_video_data_ (this,userData_);
	    Reader::attach ();
	}

	GFile *dir = g_file_get_parent (shmfile_);
	dirMonitor_ = g_file_monitor_directory (dir, G_FILE_MONITOR_NONE, NULL, NULL); 
	g_object_unref(dir);
	g_signal_connect (dirMonitor_, "changed", G_CALLBACK (Reader::file_system_monitor_change), static_cast<void *>(this)); 
    }

    void
    Reader::setSink (GstElement *pipeline, GstElement *sink)
    {
	sink_         = sink;
	pipeline_     = pipeline;

	GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
	gst_bus_set_sync_handler (bus, message_handler, static_cast<void *>(this));
	gst_object_unref (bus);
    }
    
    Reader::~Reader (){
	g_object_unref(shmfile_);
	g_object_unref(dirMonitor_);
    }

    

    void 
    Reader::attach ()
    {
	source_       = gst_element_factory_make ("shmsrc",  NULL);
	deserializer_ = gst_element_factory_make ("gdpdepay",  NULL);   
	if ( !source_ || !deserializer_ ) {
	    g_printerr ("One element could not be created. Exiting.\n");
	}
	g_object_set (G_OBJECT (source_), "socket-path", socketName_.c_str(), NULL);
	
	gst_bin_add_many (GST_BIN (pipeline_),
			  source_, deserializer_, NULL);

	deserialPad_ = gst_element_get_static_pad (deserializer_,"src");
	sinkPad_ = gst_element_get_compatible_pad (sink_,deserialPad_,GST_PAD_CAPS(deserialPad_));

	gst_element_link (source_, deserializer_);
	gst_pad_link (deserialPad_,sinkPad_);
	
	gst_element_set_state (deserializer_, GST_STATE_PLAYING);
	gst_element_set_state (source_, GST_STATE_PLAYING);
    }

    void
    Reader::detach ()
    {
	gst_element_unlink (source_, deserializer_);
	gst_pad_unlink (deserialPad_,sinkPad_);
	gst_object_unref (deserialPad_);
	gst_element_release_request_pad (sink_,sinkPad_);
	//ask for element cleaning in the main thread
	g_idle_add ((GSourceFunc) clean_source,static_cast<void *>(this));
    }

    gboolean 
    Reader::clean_source (gpointer user_data)
    {
	Reader *context = static_cast<Reader*>(user_data);
	gst_object_unref(context->sinkPad_);
	gst_element_set_state (context->deserializer_, GST_STATE_NULL);
	gst_bin_remove (GST_BIN (context->pipeline_),context->deserializer_);
	gst_element_set_state (context->source_, GST_STATE_NULL);
	gst_bin_remove (GST_BIN (context->pipeline_),context->source_); 
	return FALSE;
    }

    GstBusSyncReply Reader::message_handler (GstBus *bus, GstMessage *msg, gpointer user_data)
    {
	switch (GST_MESSAGE_TYPE (msg)) {
	case GST_MESSAGE_EOS:
	    g_print ("End of stream\n");
	    break;
	case GST_MESSAGE_ERROR: {
	    gchar  *debug;
	    GError *error;
	    Reader *context = static_cast<Reader*>(user_data);
	    gst_message_parse_error (msg, &error, &debug);
	    g_free (debug);
	    if (g_strcmp0 (GST_ELEMENT_NAME(context->source_),GST_OBJECT_NAME(msg->src)) == 0 ){
		if (error->code == GST_RESOURCE_ERROR_READ)
		    context->detach ();
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

    void
    Reader::file_system_monitor_change (GFileMonitor *      monitor,
					GFile *             file,
					GFile *             other_file,
					GFileMonitorEvent   type,
					gpointer user_data)
    {
	char *filename = g_file_get_path (file);
	Reader *context = static_cast<Reader*>(user_data);

	switch (type)
	{
	case G_FILE_MONITOR_EVENT_CREATED:
	    if (g_file_equal (file,context->shmfile_)) {
		if (! context->initialized_)
		{
		    context->initialized_ = TRUE;
		    context->on_first_video_data_ (context,context->userData_);
		}
		context->attach();
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

} //end namespace  shmdata
