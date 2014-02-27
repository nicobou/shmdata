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

#include "shmdata/osg-reader_impl.h"
#include <gst/app/gstappsink.h>
#include <iostream>

namespace shmdata
{

  OsgReader_impl::OsgReader_impl ()
    : reader_ (NULL),
      sharedVideoThread_ (NULL),
      pbo_ (NULL),
      image_ (NULL),
      bufferReady_ (false),
      debug_ (false), 
      playing_ (true), 
      width_ (-1), // will with incoming video frame 
      height_ (-1)
  {
    texture_ = new osg::Texture2D;
    texture_->setDataVariance(osg::Object::DYNAMIC);
    texture_->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
    texture_->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
    texture_->setResizeNonPowerOfTwoHint(false);

    //pbo_ = new osg::PixelBufferObject;

    /* Initialisation */
    gst_init (NULL, NULL);
    loop_ = g_main_loop_new (NULL, FALSE);
    g_thread_init (NULL);
    pipeline_ = gst_pipeline_new (NULL);
    if (pipeline_ == NULL)
      g_critical ("cannot create gstreamer pipeline");
    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
    g_log_set_default_handler (//G_LOG_DOMAIN, 
			       //(GLogLevelFlags) (G_LOG_LEVEL_WARNING | G_LOG_FLAG_FATAL | G_LOG_FLAG_RECURSION), 
			       OsgReader_impl::log_handler, 
			       static_cast<void *>(this));

    sharedVideoThread_ = g_thread_create ((GThreadFunc) OsgReader_impl::g_loop_thread, 
					  static_cast<void *>(this), 
					  FALSE, 
					  NULL);

	
  }

  bool
  OsgReader_impl::setPath (const std::string &socketPath)
  {
    if (&socketPath == NULL || socketPath == "")
      return false;
    
    if (reader_ != NULL)
      {
	g_message ("closing old reader %p",reader_);
	shmdata_base_reader_close (reader_);
	if (pipeline_ != NULL)
	  gst_element_set_state (pipeline_, GST_STATE_NULL);
	if (pipeline_ != NULL)
	  gst_object_unref (GST_OBJECT (pipeline_));
	pipeline_ = gst_pipeline_new (NULL);
	if (pipeline_ == NULL)
	  g_critical ("cannot create gstreamer pipeline");
	gst_element_set_state (pipeline_, GST_STATE_PLAYING);
      }
    
    socketName_ = new std::string (socketPath);

    
    //reader_ = shmdata_base_reader_init (socketName_->c_str(), pipeline_, 
	//				OsgReader_impl::on_first_video_data,
	//				static_cast<void *>(this));
    
    reader_ = shmdata_base_reader_new ();
    shmdata_base_reader_set_callback (reader_, OsgReader_impl::on_first_video_data, static_cast<void*>(this));
    shmdata_base_reader_set_bin (reader_, pipeline_);
    shmdata_base_reader_install_sync_handler (reader_, TRUE);
    shmdata_base_reader_start (reader_, socketName_->c_str());
    g_message ("new reader %p",reader_);
    
    return true;
  }  

 

  void
  OsgReader_impl::play ()
  {
    playing_ = true;

  }    

  void
  OsgReader_impl::pause()
  {
    playing_ = false;
  }

  OsgReader_impl::~OsgReader_impl ()
  {
    if (reader_ != NULL)
      shmdata_base_reader_close (reader_);
    if (pipeline_ != NULL)
      gst_element_set_state (pipeline_, GST_STATE_NULL);
    if (pipeline_ != NULL)
      gst_object_unref (GST_OBJECT (pipeline_));
    if (loop_ != NULL)
      g_main_loop_quit (loop_);
  }
  
  osg::Texture2D* 
  OsgReader_impl::getTexture()
  {
    return texture_;
  }

  int 
  OsgReader_impl::getWidth()
  {
    return width_;
  }

  int 
  OsgReader_impl::getHeight()
  {
    return height_;
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

    g_message ("source starting to send frames");
    GstElement *shmDisplay = gst_element_factory_make ("appsink", NULL);
    GstElement *videoConv = gst_element_factory_make ("ffmpegcolorspace", NULL);
    //in order to be dynamic, the shared video is linking to an 
    //element accepting request pad (as funnel)
    GstElement *funnel = gst_element_factory_make ("funnel", NULL);
	
    if (!shmDisplay )
      g_critical ("gstreamer element could not be created (appsink)");
    if(  !videoConv )
      g_critical ("gstreamer element could not be created (ffmpegcolorspace)");
    if( !funnel )
      g_critical ("gstreamer element could not be created (funnel)");
	
    g_object_set (G_OBJECT (shmDisplay), 
		  "caps", 
		  //		      gst_caps_from_string ("video/x-raw-rgb, bpp=16, depth=16"), 
		  gst_caps_from_string ("video/x-raw-rgb, bpp=32, depth=32, endianness=4321, red_mask=65280, green_mask=16711680, blue_mask=-16777216, alpha_mask=255"), 
		  NULL);
    g_object_set (G_OBJECT (shmDisplay), 
		  "emit-signals", TRUE, 
		  "sync", FALSE, 
		  NULL);
    g_signal_connect (shmDisplay, 
		      "new-buffer",
		      G_CALLBACK (OsgReader_impl::on_new_buffer_from_source), 
		      context);

    g_message ("frame reception handler is set");


    //element must have the same state as the pipeline
    gst_element_set_state (shmDisplay, GST_STATE_PLAYING);
    gst_element_set_state (funnel, GST_STATE_PLAYING);
    gst_element_set_state (videoConv, GST_STATE_PLAYING);

    g_message ("elements state to playing");

    gst_bin_add_many (GST_BIN (context->pipeline_), 
		      funnel, 
		      videoConv, 
		      shmDisplay, 
		      NULL);
    gst_element_link_many (funnel, 
			   videoConv, 
			   shmDisplay,
			   NULL);
    
    g_message ("elements linked");

    //now tells the shared video reader where to write the data
    shmdata_base_reader_set_sink (reader, funnel);

    g_message ("base reader sink has been set");
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
  
    context->width_  = g_value_get_int (gst_structure_get_value (imgProp,"width"));
    context->height_ = g_value_get_int (gst_structure_get_value (imgProp,"height"));

    if(context->playing_)
    {
        context->buffers_.push_back(buffer);
        context->bufferReady_ = true;
    }
  }

  void 	
  OsgReader_impl::setDebug(bool debug)
  {
    debug_ = debug;
  }

  void
  OsgReader_impl::updateImage()
  {
    if(!bufferReady_)
        return;
    if( image_ == NULL)
    {
	    image_ = new osg::ImageStream;
	    texture_->setImage(image_);
    }
	
    for(int i=0; i<buffers_.size()-1; ++i)
    {
        gst_buffer_unref(buffers_[0]);
        buffers_.erase(buffers_.begin());
    }

	image_->setImage(width_, 
	      height_, 
	      1, 
	      GL_RGBA, 
	      GL_BGRA, 
	      GL_UNSIGNED_BYTE,		      
	      GST_BUFFER_DATA (buffers_[0]),
	      osg::Image::NO_DELETE, 
	      1);
  
    texture_->dirtyTextureObject();

    bufferReady_ = false;
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
