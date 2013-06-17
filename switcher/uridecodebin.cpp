/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "uridecodebin.h"
#include "gst-utils.h"
#include <glib/gprintf.h>
#include <memory>

namespace switcher
{
  QuiddityDocumentation Uridecodebin::doc_ ("uri decoding", "uridecodebin", 
					    "decode an URI of live stream(s) to shmdata(s)");
  
  Uridecodebin::~Uridecodebin ()
  {
    destroy_uridecodebin ();
    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if ((bool) manager && g_strcmp0 ("",runtime_name_.c_str ()) != 0)
     	manager->remove_without_hook (runtime_name_);
  }
  
  bool
  Uridecodebin::init() 
  { 
    if (!GstUtils::make_element ("uridecodebin",&uridecodebin_))
      return false;

    //set the name before registering properties
    set_name (gst_element_get_name (uridecodebin_));
    
    destroy_uridecodebin ();
   
    //registering add_data_stream
    register_method("to_shmdata",
		    (void *)&to_shmdata_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("to_shmdata", 
			    "decode streams from an uri and write them to shmdatas", 
			    Method::make_arg_description ("uri", 
							  "the uri to decode",
							  NULL));
    //registering play
    register_method("play",
		    (void *)&play_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"play", 
			    (char *)"activate the runtime", 
			    Method::make_arg_description ("none",
							  NULL));

    //registering pause
    register_method("pause",
		    (void *)&pause_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"pause", 
			    (char *)"pause the runtime", 
			    Method::make_arg_description ((char *)"none",
							  NULL));

     //registering seek
    register_method("seek",
		    (void *)&seek_wrapped, 
		    Method::make_arg_type_description (G_TYPE_DOUBLE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"seek", 
			    (char *)"seek the runtime", 
			    Method::make_arg_description ((char *)"position",
							  (char *)"position in milliseconds",
							  NULL));

    //registering speed
    register_method("speed",
		    (void *)&speed_wrapped, 
		    Method::make_arg_type_description (G_TYPE_DOUBLE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"speed", 
			    (char *)"controle speed of runtime", 
			    Method::make_arg_description ((char *)"speed",
							  (char *)"1.0 is normal speed, 0.5 is half the speed and 2.0 is double speed",
							  NULL));
    return true;
  }
  
  QuiddityDocumentation 
  Uridecodebin::get_documentation ()
  {
    return doc_;
  }

  void 
  Uridecodebin::init_uridecodebin ()
  {
    if (!GstUtils::make_element ("uridecodebin",&uridecodebin_))
      {
	g_warning (" Uridecodebin::init_uridecodebin, cannot create uridecodebin");
	return;
      }
    
    media_counters_.clear ();
    main_pad_ = NULL;
    discard_next_uncomplete_buffer_ = false;
    rtpgstcaps_ = gst_caps_from_string ("application/x-rtp, media=(string)application");
    
    g_signal_connect (G_OBJECT (uridecodebin_), 
		      "pad-added", 
		      (GCallback) Uridecodebin::uridecodebin_pad_added_cb,
		      (gpointer) this);
    g_signal_connect (G_OBJECT (uridecodebin_),  
		      "no-more-pads",  
		      (GCallback) Uridecodebin::no_more_pads_cb ,  
		      (gpointer) this);      
    g_signal_connect (G_OBJECT (uridecodebin_),  
		      "source-setup",  
		      (GCallback) Uridecodebin::source_setup_cb ,  
		      (gpointer) this);      

    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "pad-removed",  
    // 		      (GCallback) Uridecodebin::pad_removed_cb ,  
    // 		      (gpointer) this);      
    g_signal_connect (G_OBJECT (uridecodebin_),  
     		      "unknown-type",  
     		      (GCallback) Uridecodebin::unknown_type_cb ,  
     		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "autoplug-continue",  
    // 		      (GCallback) Uridecodebin::autoplug_continue_cb ,  
    // 		      (gpointer) this);      
     // g_signal_connect (G_OBJECT (uridecodebin_),  
     // 		      "autoplug-factory",  
     // 		      (GCallback) Uridecodebin::autoplug_factory_cb ,  
     // 		      (gpointer) this);      
     // g_signal_connect (G_OBJECT (uridecodebin_),  
     // 		      "autoplug-sort",  
     // 		      (GCallback) Uridecodebin::autoplug_sort_cb ,  
     // 		      (gpointer) this);      
     g_signal_connect (G_OBJECT (uridecodebin_),  
     		      "autoplug-select",  
     		      (GCallback) Uridecodebin::autoplug_select_cb ,  
     		      (gpointer) this);      
    // g_signal_connect (G_OBJECT (uridecodebin_),  
    // 		      "drained",  
    // 		      (GCallback) Uridecodebin::drained_cb ,  
    // 		      (gpointer) this);      

    // g_signal_connect (G_OBJECT (uridecodebin_),  
    //  		    "drained",  
    //  		    (GCallback) uridecodebin_drained_cb ,  
    //  		    (gpointer) this);      

    g_object_set (G_OBJECT (uridecodebin_),  
       		  // "ring-buffer-max-size",(guint64)200000000, 
		  //"download",TRUE, 
		  //"use-buffering",TRUE, 
		  //"ring-buffer-max-size", 4294967295,
       		  "expose-all-streams", TRUE,
       		  "async-handling",TRUE, 
       		  //"buffer-duration",9223372036854775807, 
       		  NULL); 
  }

  void 
  Uridecodebin::destroy_uridecodebin ()
  {
    GstUtils::clean_element (uridecodebin_);
    // if (G_IS_OBJECT (uridecodebin_))
    //   {
    // 	gst_element_set_state (uridecodebin_,GST_STATE_NULL);
    // 	gst_element_get_state (uridecodebin_, NULL, NULL, GST_CLOCK_TIME_NONE);
    // 	gst_object_unref (uridecodebin_);
    //   }
  }

  void 
  Uridecodebin::no_more_pads_cb (GstElement* object, gpointer user_data)   
  {   
    //g_print ("---- no more pad\n");
    // Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  }

  void 
  Uridecodebin::unknown_type_cb (GstElement *bin, 
				 GstPad *pad, 
				 GstCaps *caps, 
				 gpointer user_data)
  {
    g_error ("Uridecodebin unknown type: %s (%s)\n", gst_caps_to_string (caps), gst_element_get_name (bin));
  }

  int 
  Uridecodebin::autoplug_select_cb (GstElement *bin, 
				    GstPad *pad, 
				    GstCaps *caps, 
				    GstElementFactory *factory, 
				    gpointer user_data)
  {
    g_debug ("uridecodebin autoplug select %s, (factory %s)",  gst_caps_to_string (caps), GST_OBJECT_NAME (factory));
    //     typedef enum {
    //   GST_AUTOPLUG_SELECT_TRY,
    //   GST_AUTOPLUG_SELECT_EXPOSE,
    //   GST_AUTOPLUG_SELECT_SKIP
    // } GstAutoplugSelectResult;

    if (g_strcmp0 (GST_OBJECT_NAME (factory), "rtpgstdepay") == 0)
      return 1; //expose
    return 0; //try
  }

  // GValueArray*
  // Uridecodebin::autoplug_factory_cb (GstElement *bin,
  // 				     GstPad          *pad,
  // 				     GstCaps         *caps,
  // 				     gpointer         user_data)
  // {
  //   g_print ("autoplug factory ---------- %s\n",gst_caps_to_string (caps));
  //   return NULL;
  // }

  // GValueArray *
  // Uridecodebin::autoplug_sort_cb (GstElement *bin,
  // 				  GstPad *pad,
  // 				  GstCaps *caps,
  // 				  GValueArray *factories,
  // 				  gpointer user_data)  
  // {
  //   g_print ("sort---------------- bin: %s\n%s\n", gst_element_get_name (bin), gst_caps_to_string (caps)); 
  //   return g_value_array_copy(factories);
  // }
  
  gboolean
  Uridecodebin::rewind (gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);

    GstQuery *query;
    gboolean res;
    query = gst_query_new_segment (GST_FORMAT_TIME);
    res = gst_element_query (context->runtime_->get_pipeline (), query);
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
      g_warning ("looping error\n");
   
    g_debug ("finish looping");
    return FALSE;
  }

  gboolean
  Uridecodebin::event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
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
    
    //g_print ("event probed (%s)\n", GST_EVENT_TYPE_NAME(event));
    

    return TRUE; 
  }


  void
  Uridecodebin::pad_to_shmdata_writer (GstElement *bin, GstPad *pad)
  {
    //g_print ("---- (%s)\n", gst_caps_to_string (gst_pad_get_caps (pad)));

    //detecting type of media
    const gchar *padname;
    if (0 == g_strcmp0 ("ANY", gst_caps_to_string (gst_pad_get_caps (pad))))
      padname = "custom";
    else
      padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));

    g_debug ("uridecodebin new pad name is %s\n",padname);
    
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
     if (media_counters_.contains (std::string (padname_splitted[0])))
       {
	 count = media_counters_. lookup (std::string (padname_splitted[0]));
	 //g_print ("------------- count %d\n", count);
	 count = count+1;
       }
     media_counters_.replace (std::string (padname_splitted[0]), count);

     gchar media_name[256];
     g_sprintf (media_name,"%s_%d",padname_splitted[0],count);
     g_debug ("uridecodebin: new media %s %d\n",media_name, count );
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
  Uridecodebin::gstrtpdepay_buffer_probe_cb (GstPad * pad, GstMiniObject * mini_obj, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    
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
  Uridecodebin::gstrtpdepay_event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);

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
  Uridecodebin::uridecodebin_pad_added_cb (GstElement* object, GstPad *pad, gpointer user_data)   
  {   
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
   

    // g_print ("------------- caps 1 %s \n-------------- caps 2 %s\n",
    // 	     gst_caps_to_string (context->gstrtpcaps_), 
    // 	     gst_caps_to_string (gst_pad_get_caps (pad)));
   
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
				  G_CALLBACK (Uridecodebin::gstrtpdepay_buffer_probe_cb),
				  context);
	gst_object_unref (depaysrcpad);

	gst_bin_add (GST_BIN (context->bin_), rtpgstdepay);
	GstPad *sinkpad = gst_element_get_static_pad (rtpgstdepay, "sink");
	//adding a probe for handling loss messages from rtpbin
	gst_pad_add_event_probe (sinkpad, (GCallback) Uridecodebin::gstrtpdepay_event_probe_cb, context);   

	GstUtils::check_pad_link_return (gst_pad_link (pad, sinkpad));
	gst_object_unref (sinkpad);
	GstPad *srcpad = gst_element_get_static_pad (rtpgstdepay, "src");
	GstUtils::sync_state_with_parent (rtpgstdepay);
	gst_element_get_state (rtpgstdepay, NULL, NULL, GST_CLOCK_TIME_NONE);
	context->pad_to_shmdata_writer (context->bin_, srcpad);
	//gst_object_unref (srcpad);
      }
    else
      context->pad_to_shmdata_writer (context->bin_, pad);
  }   

  void 
  Uridecodebin::source_setup_cb (GstElement *uridecodebin, GstElement *source, gpointer user_data)
  {
    //Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    g_debug ("source %s %s\n",  GST_ELEMENT_NAME(source), G_OBJECT_CLASS_NAME (G_OBJECT_GET_CLASS (source)));
  }

  gboolean
  Uridecodebin::to_shmdata_wrapped (gpointer uri, 
				    gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  
    if (context->to_shmdata ((char *)uri))
      return TRUE;
    else
      return FALSE;
  }

  bool
  Uridecodebin::to_shmdata (std::string uri)
  {
    destroy_uridecodebin ();
    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if ((bool) manager)
      {
	manager->remove_without_hook (runtime_name_);
	runtime_name_ = manager->create_without_hook ("runtime");
	Quiddity::ptr quidd = manager->get_quiddity (runtime_name_);
	Runtime::ptr runtime = std::dynamic_pointer_cast<Runtime> (quidd);
	if(runtime)
	  set_runtime(runtime);
	else
	  g_warning ("Uridecodebin::to_shmdata: unable to use a custom runtime");
      }
    else
      return false;
    
    reset_bin ();
    init_uridecodebin ();
    g_debug ("------------------------- to_shmdata set uri %s", uri.c_str ());
    g_object_set (G_OBJECT (uridecodebin_), "uri", uri.c_str (), NULL); 

    gst_bin_add (GST_BIN (bin_), uridecodebin_);
    GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (uridecodebin_);
    return true;
  }

  gboolean
  Uridecodebin::play_wrapped (gpointer unused, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if (! (bool) manager)
      return FALSE;
    Quiddity::ptr quidd = manager->get_quiddity (context->runtime_name_);
    Runtime::ptr runtime = std::dynamic_pointer_cast<Runtime> (quidd);
    if(!runtime)
      return FALSE;
    runtime->play ();
    return TRUE;
  }
  
  gboolean
  Uridecodebin::pause_wrapped (gpointer unused, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if (! (bool) manager)
      return FALSE;
    Quiddity::ptr quidd = manager->get_quiddity (context->runtime_name_);
    Runtime::ptr runtime = std::dynamic_pointer_cast<Runtime> (quidd);
    if(!runtime)
      return FALSE;
    runtime->pause ();
    return TRUE;
  }

  gboolean
  Uridecodebin::seek_wrapped (gdouble position, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if (! (bool) manager)
      return FALSE;
    Quiddity::ptr quidd = manager->get_quiddity (context->runtime_name_);
    Runtime::ptr runtime = std::dynamic_pointer_cast<Runtime> (quidd);
    if(!runtime)
      return FALSE;
    runtime->seek (position);
    return TRUE;
  }

  gboolean
  Uridecodebin::speed_wrapped (gdouble speed, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if (! (bool) manager)
      return FALSE;
    Quiddity::ptr quidd = manager->get_quiddity (context->runtime_name_);
    Runtime::ptr runtime = std::dynamic_pointer_cast<Runtime> (quidd);
    if(!runtime)
      return FALSE;
    runtime->speed (speed);
    return TRUE;
  }

}
