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

#include "shmdata/osg-reader_impl.h"
#include <gst/app/gstappsink.h>

namespace shmdata
{

    OsgReader_impl::OsgReader_impl ()
	: last_buffer_ (NULL), debug_ (false)
    {
	texture_ = new osg::Texture2D;
	texture_->setDataVariance(osg::Object::DYNAMIC);
	texture_->setResizeNonPowerOfTwoHint(false);
    }

    void
    OsgReader_impl::start (const std::string &socketPath)
    {

        /* Initialisation */
	gst_init (NULL, NULL);
	pipeline_ = gst_pipeline_new (NULL);

	socketName_ = &socketPath;

	g_log_set_default_handler (//G_LOG_DOMAIN, 
//				   (GLogLevelFlags) (G_LOG_LEVEL_WARNING | G_LOG_FLAG_FATAL | G_LOG_FLAG_RECURSION), 
				   OsgReader_impl::log_handler, 
				   static_cast<void *>(this));

	shmdata_base_reader_init (socketName_->c_str(), pipeline_, 
	  		     OsgReader_impl::on_first_video_data,
	  		     static_cast<void *>(this));
    

	loop_ = g_main_loop_new (NULL, FALSE);
	g_thread_init (NULL);
	sharedVideoThread_ = g_thread_create ((GThreadFunc) OsgReader_impl::g_loop_thread, 
	  				      static_cast<void *>(this), 
	  				      FALSE, 
	  				      NULL);
    }    

    OsgReader_impl::~OsgReader_impl ()
    {
	//TODO call shmdata_base_reader_close
	gst_element_set_state (pipeline_, GST_STATE_NULL);
	gst_object_unref (GST_OBJECT (pipeline_));
	g_main_loop_quit (loop_);
        //delete texture_; //desctructor protected
	g_debug ("object deleted");
    }
    
    osg::Texture2D* 
    OsgReader_impl::getTexture()
    {
	return texture_;
    }


    void
    OsgReader_impl::g_loop_thread (gpointer user_data)
    {
	OsgReader_impl *context = static_cast<OsgReader_impl*>(user_data);
	g_main_loop_run (context->loop_);
    }


    void
    OsgReader_impl::on_first_video_data (shmdata_base_reader_t *reader, void *user_data)
    {

	OsgReader_impl *context = static_cast<OsgReader_impl*>(user_data);

	gst_element_set_state (context->pipeline_, GST_STATE_PLAYING);

	g_message ("source stating to send frames");
	GstElement *shmDisplay = gst_element_factory_make ("appsink", NULL);
	GstElement *videoConv = gst_element_factory_make ("ffmpegcolorspace", NULL);

	g_object_set (G_OBJECT (shmDisplay), 
		      "caps", 
		      //		      gst_caps_from_string ("video/x-raw-rgb, bpp=16, depth=16"), 
		      gst_caps_from_string ("video/x-raw-rgb, bpp=32, depth=32, endianness=4321, red_mask=65280, green_mask=16711680, blue_mask=-16777216, alpha_mask=255"), 
		      NULL);
	g_object_set (G_OBJECT (shmDisplay), 
		      "emit-signals", 
		      TRUE, "sync", 
		      FALSE, 
		      NULL);
	g_signal_connect (shmDisplay, 
			  "new-buffer",
			  G_CALLBACK (OsgReader_impl::on_new_buffer_from_source), 
			  context);

	//in order to be dynamic, the shared video is linking to an 
	//element accepting request pad (as funnel of videomixer)
	GstElement *funnel = gst_element_factory_make ("funnel", NULL);
	g_object_set (G_OBJECT (shmDisplay), 
		      "sync", 
		      FALSE, 
		      NULL);
	if (!shmDisplay )
	    g_critical ("gstreamer element could not be created (appsink)");
	if(  !videoConv )
	    g_critical ("gstreamer element could not be created (ffmpegcolorspace)");
	if( !funnel )
	    g_critical ("gstreamer element could not be created (funnel)");

	//element must have the same state as the pipeline
	gst_element_set_state (shmDisplay, GST_STATE_PLAYING);
	gst_element_set_state (funnel, GST_STATE_PLAYING);
	gst_element_set_state (videoConv, GST_STATE_PLAYING);
	gst_bin_add_many (GST_BIN (context->pipeline_), funnel, videoConv, shmDisplay, NULL);
	gst_element_link_many (funnel, videoConv, shmDisplay,NULL);
    
	//now tells the shared video reader where to write the data
	shmdata_base_reader_set_sink (reader, 
				 funnel);
    }

    /* called when the appsink notifies us that there is a new buffer ready for
     * processing */
    void
    OsgReader_impl::on_new_buffer_from_source (GstElement * elt, gpointer user_data)
    {
	GstBuffer *buffer;
	OsgReader_impl *context = static_cast<OsgReader_impl*>(user_data);

	buffer = gst_app_sink_pull_buffer (GST_APP_SINK (elt));
	GstStructure *imgProp=gst_caps_get_structure (GST_BUFFER_CAPS(buffer),0);
  
	int curWidth = g_value_get_int (gst_structure_get_value (imgProp,"width"));
	int curHeight = g_value_get_int (gst_structure_get_value (imgProp,"height"));

	osg::Image *img = new osg::Image;
	img->setOrigin(osg::Image::TOP_LEFT); 
	img->setImage(curWidth, 
		      curHeight, 
		      0, 
		      GL_RGBA, 
		      GL_BGRA, 
		      GL_UNSIGNED_BYTE,		      
		      //GL_UNSIGNED_SHORT_5_6_5, 
		      GST_BUFFER_DATA (buffer),
		      osg::Image::NO_DELETE, 
		      1);

	context->texture_->setImage(img);

	if (context->last_buffer_ != NULL)  
	    gst_buffer_unref (context->last_buffer_);
	context->last_buffer_ = buffer;
    }

    void 
    OsgReader_impl::setDebug(bool debug)
    {
	debug_ = debug;
    }

    void
    OsgReader_impl::log_handler (const gchar *log_domain, 
				 GLogLevelFlags log_level,
				 const gchar *message,
				 gpointer user_data)
    {
	
	if ( g_strcmp0 (log_domain,G_LOG_DOMAIN) == 0 )
	{
	    OsgReader_impl *context = static_cast<OsgReader_impl*>(user_data);

	    switch (log_level) {
	    case G_LOG_LEVEL_ERROR:
	 	if (context->debug_) 
	 	    g_print ("%s, ERROR: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    case G_LOG_LEVEL_CRITICAL:
	 	if (context->debug_) 
	 	    g_print ("%s, CRITICAL: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    case G_LOG_LEVEL_WARNING:
	 	if (context->debug_) 
	 	    g_print ("%s, WARNING: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    case G_LOG_LEVEL_MESSAGE:
	 	if (context->debug_) 
	 	    g_print ("%s, MESSAGE: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    case G_LOG_LEVEL_INFO:
	 	if (context->debug_) 
	 	    g_print ("%s, INFO: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    case G_LOG_LEVEL_DEBUG:
	 	if (context->debug_) 
	 	    g_print ("%s, DEBUG: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    default:
	 	if (context->debug_) 
	 	    g_print ("%s: %s\n",G_LOG_DOMAIN,message);
	 	break;
	    }
	}
    }
    

}//end namespace shmdata
