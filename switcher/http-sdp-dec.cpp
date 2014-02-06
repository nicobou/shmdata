/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "http-sdp-dec.h"
#include "gst-utils.h"
#include <glib/gprintf.h>
#include <memory>

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(HTTPSDPDec,
				       "HTTP/SDP Decoder",
				       "network source", 
				       "decode an sdp-described stream deliver through http and make shmdatas",
				       "LGPL",
				       "httpsdpdec", 
				       "Nicolas Bouillot");
  
  HTTPSDPDec::HTTPSDPDec () :
    souphttpsrc_ (NULL),
    sdpdemux_ (NULL),
    decodebins_ (),
    media_counters_ (),
    main_pad_ (NULL),
    rtpgstcaps_ (NULL),
    discard_next_uncomplete_buffer_ (false),
    on_error_command_ (NULL)
  {}

  HTTPSDPDec::~HTTPSDPDec ()
  {
    destroy_httpsdpdec ();
  }
  
  bool
  HTTPSDPDec::init_segment () 
  { 
    GstElement *decodebin = NULL;
    if (!GstUtils::make_element ("souphttpsrc", &souphttpsrc_)
	|| !GstUtils::make_element ("sdpdemux", &sdpdemux_)
	|| !GstUtils::make_element ("decodebin2", &decodebin))
      return false;
    decodebins_.push_back (decodebin);
    destroy_httpsdpdec ();
 
    install_method ("To Shmdata",
		    "to_shmdata", 
		    "get raw streams from an sdp description distributed over http and write them to shmdatas", 
		    "success or fail",
		    Method::make_arg_description ("URL",
						  "url", 
						  "the url to the sdp file",
						  NULL),
		    (Method::method_ptr) &to_shmdata_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);
    return true;
  }

  void 
  HTTPSDPDec::init_httpsdpdec ()
  {
    if (!GstUtils::make_element ("souphttpsrc", &souphttpsrc_)
	|| !GstUtils::make_element ("sdpdemux", &sdpdemux_))
      {
	g_warning ("HTTPSDPDec::init_httpsdpdec, cannot create httpsdpdec elements");
	return;
      }

    media_counters_.clear ();
    main_pad_ = NULL;
    
    discard_next_uncomplete_buffer_ = false;
    rtpgstcaps_ = gst_caps_from_string ("application/x-rtp, media=(string)application");

    g_signal_connect (G_OBJECT (sdpdemux_), 
		      "pad-added", 
		      (GCallback) HTTPSDPDec::httpsdpdec_pad_added_cb,
		      (gpointer) this);
    g_signal_connect (G_OBJECT (sdpdemux_),  
		      "no-more-pads",  
		      (GCallback) HTTPSDPDec::no_more_pads_cb ,  
		      (gpointer) this);      
  }

  void 
  HTTPSDPDec::destroy_httpsdpdec ()
  {
    if (NULL != souphttpsrc_)
      GstUtils::clean_element (souphttpsrc_);
    souphttpsrc_ = NULL;
    if (NULL != sdpdemux_)
      GstUtils::clean_element (sdpdemux_);
    sdpdemux_ = NULL;
    clean_on_error_command ();
  }

  void 
  HTTPSDPDec::clean_on_error_command ()
  {
    if (on_error_command_ != NULL)
      {
	delete on_error_command_;
	on_error_command_ = NULL;
      }
  }

  void 
  HTTPSDPDec::no_more_pads_cb (GstElement */*object*/, gpointer /*user_data*/)   
  {   
  }

  void 
  HTTPSDPDec::unknown_type_cb (GstElement *bin, 
			       GstPad */*pad*/, 
			       GstCaps *caps, 
			       gpointer /*user_data*/)
  {
    g_debug ("HTTPSDPDec unknown type: %s (%s)\n", gst_caps_to_string (caps), gst_element_get_name (bin));
  }

  int 
  HTTPSDPDec::autoplug_select_cb (GstElement */*bin*/, 
				  GstPad *pad, 
				  GstCaps */*caps*/, 
				  GstElementFactory *factory, 
				  gpointer /*user_data*/)
  {
    //g_print ("\n --- httpsdpdec autoplug select %s, (factory %s)\n\n",  gst_caps_to_string (caps), GST_OBJECT_NAME (factory));
    //     typedef enum {
    //   GST_AUTOPLUG_SELECT_TRY,
    //   GST_AUTOPLUG_SELECT_EXPOSE,
    //   GST_AUTOPLUG_SELECT_SKIP
    // } GstAutoplugSelectResult;

    if (g_strcmp0 (GST_OBJECT_NAME (factory), "rtpgstdepay") == 0)
      {
	int return_val = 1;
	const GValue *val = gst_structure_get_value (gst_caps_get_structure(gst_pad_get_caps (pad),0),
						     "caps");
	
	gsize taille = 256;
	guchar *string_caps = g_base64_decode (g_value_get_string (val),
					       &taille);

	gchar *string_caps_char = g_strdup_printf ("%s", string_caps); 

	if (g_str_has_prefix (string_caps_char, "audio/") 
	    || g_str_has_prefix (string_caps_char, "video/") )
	  return_val = 0;

	g_free (string_caps_char);
	g_free (string_caps);
	return return_val; //expose
      }
    return 0; //try
  }

  // GValueArray*
  // HTTPSDPDec::autoplug_factory_cb (GstElement *bin,
  // 				     GstPad          *pad,
  // 				     GstCaps         *caps,
  // 				     gpointer         user_data)
  // {
  //   g_print ("autoplug factory ---------- %s\n",gst_caps_to_string (caps));
  //   return NULL;
  // }

  // GValueArray *
  // HTTPSDPDec::autoplug_sort_cb (GstElement *bin,
  // 				  GstPad *pad,
  // 				  GstCaps *caps,
  // 				  GValueArray *factories,
  // 				  gpointer user_data)  
  // {
  //   g_print ("sort---------------- bin: %s\n%s\n", gst_element_get_name (bin), gst_caps_to_string (caps)); 
  //   return g_value_array_copy(factories);
  // }
  
  gboolean
  HTTPSDPDec::rewind (gpointer user_data)
  {
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    GstQuery *query;
    gboolean res;
    query = gst_query_new_segment (GST_FORMAT_TIME);
    res = gst_element_query (context->get_pipeline (), query);
    gdouble rate = -2.0;
    gint64 start_value = -2.0;
    gint64 stop_value = -2.0;
    if (res) {
      gst_query_parse_segment (query, &rate, NULL, &start_value, &stop_value);
      // g_print ("rate = %f start = %"GST_TIME_FORMAT" stop = %"GST_TIME_FORMAT"\n", 
      // 	       rate,
      // 	       GST_TIME_ARGS (start_value),
      // 	       GST_TIME_ARGS (stop_value));
    }
    else {
      g_debug ("duration query failed...");
    }
    gst_query_unref (query);
    gboolean ret;
    ret = gst_element_seek (GST_ELEMENT (gst_pad_get_parent (context->main_pad_)),
			    rate,  
			    GST_FORMAT_TIME,  
			    (GstSeekFlags) (GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE), 
			    //| GST_SEEK_FLAG_SKIP 
			    //| GST_SEEK_FLAG_KEY_UNIT, //using key unit is breaking synchronization 
			    GST_SEEK_TYPE_SET,  
			    0.0 * GST_SECOND,  
			    GST_SEEK_TYPE_NONE,  
			    GST_CLOCK_TIME_NONE);  
    if (!ret)
      g_debug ("looping error\n");
    g_debug ("finish looping");
    return FALSE;
  }

  gboolean
  HTTPSDPDec::event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data)
  {
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    if (GST_EVENT_TYPE (event) == GST_EVENT_EOS) { 
      //g_print ("EOS caught and disabled \n");
      // g_print ("----- pad with EOS %s:%s, src: %p %s\n",
      // 	       GST_DEBUG_PAD_NAME (pad),GST_EVENT_SRC(event), gst_element_get_name (GST_EVENT_SRC(event)));

      if (pad == context->main_pad_)
	{
	  //g_print ("main pad !@! \n");
	  GstUtils::g_idle_add_full_with_context (context->get_g_main_context (),
						  G_PRIORITY_DEFAULT_IDLE,
						  (GSourceFunc) rewind,   
						  (gpointer)context,
						  NULL);   
	}
      return FALSE;
    }  

    if (GST_EVENT_TYPE (event) == GST_EVENT_FLUSH_START || 
	GST_EVENT_TYPE (event) == GST_EVENT_FLUSH_STOP /*||
	GST_EVENT_TYPE (event) == GST_EVENT_LATENCY */) 
      {
	return FALSE;
      }
    return TRUE; 
  }

  void
  HTTPSDPDec::pad_to_shmdata_writer (GstElement *bin, GstPad *pad)
  {
    //detecting type of media
    const gchar *padname;
    if (0 == g_strcmp0 ("ANY", gst_caps_to_string (gst_pad_get_caps (pad))))
      padname = "custom";
    else
      padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));

    g_debug ("httpsdpdec new pad name is %s\n",padname);
    
    GstElement *identity;
    GstUtils::make_element ("identity", &identity);
    g_object_set (identity, 
     		  "sync", TRUE, 
     		  "single-segment", TRUE,
     		  NULL);
    GstElement *funnel;
    GstUtils::make_element ("funnel", &funnel);
    
    gst_bin_add_many (GST_BIN (bin), identity, funnel, NULL);
    GstUtils::link_static_to_request (pad, funnel);
    gst_element_link (funnel, identity);

    //GstUtils::wait_state_changed (bin);
     GstUtils::sync_state_with_parent (identity);
     GstUtils::sync_state_with_parent (funnel);
    
     //probing eos   
     GstPad *srcpad = gst_element_get_static_pad (funnel, "src");
     if (main_pad_ == NULL)
       main_pad_ = srcpad;//saving first pad for looping
     gst_pad_add_event_probe (srcpad, (GCallback) event_probe_cb, this);   
     gst_object_unref (srcpad);

     //giving a name to the stream
     gchar **padname_splitted = g_strsplit_set (padname, "/",-1);
     //counting 
    int count = 0;
    auto it = media_counters_.find (std::string (padname_splitted[0]));
    if (media_counters_.end () != it)
	count = ++(it->second);
    else
      {
	std::string media_type ("unknown");
	if (NULL != padname_splitted[0])
	  media_type = padname_splitted[0];
	media_counters_[media_type] = count;
      }
     gchar media_name[256];
     g_sprintf (media_name,"%s-%d",padname_splitted[0],count);
     g_debug ("httpsdpdec: new media %s %d\n",media_name, count );
     g_strfreev(padname_splitted);

     //creating a shmdata
     ShmdataWriter::ptr connector;
     connector.reset (new ShmdataWriter ());
     std::string connector_name = make_file_name (media_name);
     connector->set_path (connector_name.c_str());
     GstCaps *caps = gst_pad_get_caps_reffed (pad);

     connector->plug (bin, identity, caps);

     if (G_IS_OBJECT (caps))
       gst_object_unref (caps);
     register_shmdata_writer (connector);

     g_message ("%s created a new shmdata writer (%s)", 
     	       get_nick_name ().c_str(), 
     	       connector_name.c_str ());
  }

  gboolean 
  HTTPSDPDec::gstrtpdepay_buffer_probe_cb (GstPad */*pad*/, 
					   GstMiniObject */*mini_obj*/, 
					   gpointer user_data)
  {
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    /* if (GST_IS_BUFFER (mini_obj)) */
    /*   { */
    /*     GstBuffer *buffer = GST_BUFFER_CAST (mini_obj); */
    /*     g_print ("data size %d \n", */
    /* 	       GST_BUFFER_SIZE (buffer)); */
    /*   } */
    if (context->discard_next_uncomplete_buffer_ == true)
      {
	g_debug ("discarding uncomplete custom frame due to a network loss");
	context->discard_next_uncomplete_buffer_ = false;
	return FALSE;// drop buffer
      }
    else
      return TRUE; // pass buffer
  }
  
  gboolean
  HTTPSDPDec::gstrtpdepay_event_probe_cb (GstPad */*pad*/,
					  GstEvent *event, 
					  gpointer user_data)
  {
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    if (GST_EVENT_TYPE (event) == GST_EVENT_CUSTOM_DOWNSTREAM) 
      {
	const GstStructure *s;
	s = gst_event_get_structure (event);
	//g_debug ("event probed (%s)\n", gst_structure_get_name (s));
	if (gst_structure_has_name (s, "GstRTPPacketLost")) 
	  context->discard_next_uncomplete_buffer_ = true;
	return FALSE;
      }
    return TRUE; 
  }

  void 
  HTTPSDPDec::httpsdpdec_pad_added_cb (GstElement */*object*/, 
				       GstPad *pad, 
				       gpointer user_data)   
  {   
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    GstElement *decodebin = NULL;
    GstUtils::make_element ("decodebin2", &decodebin);
    gst_bin_add_many (GST_BIN (context->bin_), 
		      decodebin,
		      NULL);
    GstPad *sinkpad = gst_element_get_static_pad (decodebin, "sink");
    GstUtils::check_pad_link_return (gst_pad_link (pad, sinkpad));
    g_signal_connect (G_OBJECT (decodebin), 
		      "pad-added", 
		      (GCallback) HTTPSDPDec::decodebin_pad_added_cb,
		      (gpointer) context);
    g_signal_connect (G_OBJECT (decodebin),  
     		      "autoplug-select",  
     		      (GCallback) HTTPSDPDec::autoplug_select_cb ,  
     		      (gpointer) context);      
    GstUtils::sync_state_with_parent (decodebin);
    context->decodebins_.push_back (decodebin); 
  }   

  void 
  HTTPSDPDec::decodebin_pad_added_cb (GstElement* object, GstPad *pad, gpointer user_data)   
  {   
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    
    if (gst_caps_can_intersect (context->rtpgstcaps_,
				gst_pad_get_caps (pad)))
      {
	//asking rtpbin to send an event when a packet is lost (do-lost property)
	GstUtils::set_element_property_in_bin (object, "gstrtpbin", "do-lost", TRUE);
	
	g_message ("custom rtp stream found");
	GstElement *rtpgstdepay;
	GstUtils::make_element ("rtpgstdepay", &rtpgstdepay);

	//adding a probe for discarding uncomplete packets
	GstPad *depaysrcpad = gst_element_get_static_pad (rtpgstdepay, "src");
	gst_pad_add_buffer_probe (depaysrcpad,
				  G_CALLBACK (HTTPSDPDec::gstrtpdepay_buffer_probe_cb),
				  context);
	gst_object_unref (depaysrcpad);

	gst_bin_add (GST_BIN (context->bin_), rtpgstdepay);
	GstPad *sinkpad = gst_element_get_static_pad (rtpgstdepay, "sink");
	//adding a probe for handling loss messages from rtpbin
	gst_pad_add_event_probe (sinkpad, (GCallback) HTTPSDPDec::gstrtpdepay_event_probe_cb, context);   

	GstUtils::check_pad_link_return (gst_pad_link (pad, sinkpad));
	gst_object_unref (sinkpad);
	GstPad *srcpad = gst_element_get_static_pad (rtpgstdepay, "src");
	GstUtils::sync_state_with_parent (rtpgstdepay);
	gst_element_get_state (rtpgstdepay, NULL, NULL, GST_CLOCK_TIME_NONE);

	context->pad_to_shmdata_writer (context->bin_, srcpad);
	gst_object_unref (srcpad);
	return;
      }
    
    context->pad_to_shmdata_writer (context->bin_, pad);
  }

  void 
  HTTPSDPDec::source_setup_cb (GstElement */*httpsdpdec*/, 
			       GstElement *source, 
			       gpointer /*user_data*/)
  {
    //HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
    g_debug ("source %s %s\n",  GST_ELEMENT_NAME(source), G_OBJECT_CLASS_NAME (G_OBJECT_GET_CLASS (source)));
  }

  gboolean
  HTTPSDPDec::to_shmdata_wrapped (gpointer uri, 
				    gpointer user_data)
  {
    HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  
    if (context->to_shmdata ((char *)uri))
      return TRUE;
    else
      return FALSE;
  }

  bool
  HTTPSDPDec::to_shmdata (std::string uri)
  {
    destroy_httpsdpdec ();
    reset_bin ();
    init_httpsdpdec ();
    clean_on_error_command ();
    on_error_command_ = new QuiddityCommand ();
    on_error_command_->id_ = QuiddityCommand::invoke;
    on_error_command_->time_ = 1000; // 1 second
    on_error_command_->add_arg (get_nick_name ());
    on_error_command_->add_arg ("to_shmdata");
    std::vector<std::string> vect_arg;
    vect_arg.push_back (uri);
    on_error_command_->set_vector_arg (vect_arg);

    g_object_set_data (G_OBJECT (sdpdemux_), 
     		       "on-error-command",
     		       (gpointer)on_error_command_);
    
    g_debug ("httpsdpdec: to_shmdata set uri %s", uri.c_str ());

    g_object_set (G_OBJECT (souphttpsrc_), "location", uri.c_str (), NULL); 

    gst_bin_add_many (GST_BIN (bin_), 
		      souphttpsrc_, 
		      sdpdemux_,
		      NULL);
    
    gst_element_link (souphttpsrc_, sdpdemux_);

    //GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (souphttpsrc_);
    GstUtils::sync_state_with_parent (sdpdemux_);
    return true;
  }
}
