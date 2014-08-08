/*
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

#include "uridecodebin.h"
#include "gst-utils.h"
#include <glib/gprintf.h>
#include <memory>
#include "quiddity-command.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Uridecodebin,
				       "Media Player (URI)",
				       "uri source", 
				       "decode an URI and writes to shmdata(s)",
				       "LGPL",
				       "uridecodebin", 
				       "Nicolas Bouillot");
  
  Uridecodebin::~Uridecodebin ()
  {
    g_free (uri_);
    destroy_uridecodebin ();
  }

  Uridecodebin::Uridecodebin () :
    uridecodebin_ (nullptr),
    media_counters_ (),
    main_pad_ (nullptr),
    rtpgstcaps_ (nullptr),
    discard_next_uncomplete_buffer_ (false),
    on_error_command_ (nullptr),
    custom_props_ (new CustomPropertyHelper ()),
    loop_prop_ (nullptr),
    loop_ (false),
    playing_prop_ (nullptr),
    playing_ (true),
    uri_spec_ (nullptr),
    uri_ (g_strdup ("")) 
  {}
  
  bool
  Uridecodebin::init_gpipe () 
  { 
    if (!GstUtils::make_element ("uridecodebin",&uridecodebin_))
      return false;
    install_play_pause ();
    //install_seek ();
    uri_spec_ = 
      custom_props_->make_string_property ("uri", 
					   "URI To Be Redirected Into Shmdata(s)",
					   "",
					   (GParamFlags) G_PARAM_READWRITE,
					   Uridecodebin::set_uri,
					   Uridecodebin::get_uri,
					   this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				uri_spec_, 
				"uri",
				"URI");
    loop_prop_ = 
      custom_props_->make_boolean_property ("loop", 
					    "loop media",
					    (gboolean)FALSE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Uridecodebin::set_loop,
					    Uridecodebin::get_loop,
					    this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				loop_prop_, 
				"loop",
				"Looping");
    return true;
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
    main_pad_ = nullptr;
    //discard_next_uncomplete_buffer_ = false;
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
    g_signal_connect (G_OBJECT (uridecodebin_),  
    		      "autoplug-continue",  
    		      (GCallback) Uridecodebin::autoplug_continue_cb ,  
    		      (gpointer) this);      
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
       		  nullptr); 
  }

  void 
  Uridecodebin::destroy_uridecodebin ()
  {
    GstUtils::clean_element (uridecodebin_);
    clean_on_error_command ();
    clear_shmdatas ();
  }

  void 
  Uridecodebin::clean_on_error_command ()
  {
    if (on_error_command_ != nullptr)
      {
	delete on_error_command_;
	on_error_command_ = nullptr;
      }
  }


  void 
  Uridecodebin::no_more_pads_cb (GstElement */*object*/, 
				 gpointer /*user_data*/)   
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
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    g_warning ("Uridecodebin unknown type: %s (%s)\n", gst_caps_to_string (caps), gst_element_get_name (bin));
    context->pad_to_shmdata_writer (context->bin_, pad);
  }

  gboolean
  sink_factory_filter (GstPluginFeature* feature, gpointer data)
  {
    GstCaps* caps = (GstCaps*)data;

    if (!GST_IS_ELEMENT_FACTORY (feature))
        return FALSE;

    const GList* static_pads = gst_element_factory_get_static_pad_templates (GST_ELEMENT_FACTORY (feature));
    int not_any_number = 0;
    for (GList* item = (GList*)static_pads; item; item = item->next)
    {
        GstStaticPadTemplate* padTemplate = (GstStaticPadTemplate*)item->data;
        GstPadTemplate* pad = gst_static_pad_template_get (padTemplate);
        GstCaps* padCaps = gst_pad_template_get_caps (pad);
        if (!gst_caps_is_any (padCaps))
            not_any_number++;
    }
    if (not_any_number == 0)
        return FALSE;

    if (!gst_element_factory_list_is_type (GST_ELEMENT_FACTORY (feature), GST_ELEMENT_FACTORY_TYPE_DECODABLE))
        return FALSE;

    if (!gst_element_factory_can_sink_caps (GST_ELEMENT_FACTORY (feature), caps))
        return FALSE;

    return TRUE;
  }

  int 
  Uridecodebin::autoplug_continue_cb (GstElement */*bin*/, 
				    GstPad */*pad*/, 
				    GstCaps *caps, 
				    gpointer /*user_data*/)
  {
    GList *list = gst_registry_feature_filter (gst_registry_get_default(),
                                (GstPluginFeatureFilter) sink_factory_filter,
                                FALSE, caps);
    int length = g_list_length (list);
    gst_plugin_feature_list_free (list);
    if (length == 0)
        return 0;
    return 1;
  }

  int 
  Uridecodebin::autoplug_select_cb (GstElement */*bin*/, 
				    GstPad */*pad*/, 
				    GstCaps *caps, 
				    GstElementFactory *factory, 
				    gpointer /*user_data*/)
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
  //   return nullptr;
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
  Uridecodebin::process_eos (gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    GstQuery *query;
    gboolean res;
    query = gst_query_new_segment (GST_FORMAT_TIME);
    res = gst_element_query (context->get_pipeline (), query);
    gdouble rate = 1.0;
    gint64 start_value = -2.0;
    gint64 stop_value = -2.0;
    if (res) {
      gst_query_parse_segment (query, &rate, nullptr, &start_value, &stop_value);
      // g_print ("rate = %f start = %"GST_TIME_FORMAT" stop = %"GST_TIME_FORMAT"\n", 
      // 	       rate,
      // 	       GST_TIME_ARGS (start_value),
      // 	       GST_TIME_ARGS (stop_value));
    }
    else {
      g_debug ("duration query failed...");
    }
    gst_query_unref (query);
   
    if (!context->loop_)
      {
	context->play (FALSE);
      }

    gboolean ret = FALSE;
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
    return FALSE;
  }

  gboolean
  Uridecodebin::event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    if (GST_EVENT_TYPE (event) == GST_EVENT_EOS) { 
      // g_print ("----- pad with EOS %s:%s, src: %p %s\n",
      //  	       GST_DEBUG_PAD_NAME (pad),GST_EVENT_SRC(event), gst_element_get_name (GST_EVENT_SRC(event)));
      
      if (pad == context->main_pad_)
	{
	  GstUtils::g_idle_add_full_with_context (context->get_g_main_context (),
						  G_PRIORITY_DEFAULT_IDLE,
						  (GSourceFunc) process_eos,   
						  (gpointer)context,
						  nullptr);   
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
  Uridecodebin::on_handoff_cb (GstElement* /*object*/,
			     GstBuffer* buf,
			     GstPad* pad,
			     gpointer user_data)
  {
    ShmdataAnyWriter *writer = static_cast <ShmdataAnyWriter *> (user_data);

    if (!writer->started ())
      {
	      GstCaps *caps = gst_pad_get_negotiated_caps (pad);
	      gchar *string_caps = gst_caps_to_string (caps);
	      g_debug ("%s\n", string_caps);
	      writer->set_data_type (string_caps);
	      writer->start ();
	      g_free (string_caps);
	      gst_caps_unref (caps);
      }
    else
      {
	      GstBuffer *buftmp = gst_buffer_copy (buf);
	      writer->push_data (GST_BUFFER_DATA (buftmp),
	     		    GST_BUFFER_SIZE (buftmp),
	     		    GST_BUFFER_TIMESTAMP (buftmp),
	     		    release_buf,
	     		    buftmp);
      }
  }

  void
  Uridecodebin::release_buf (void *user_data)
  {
    GstBuffer *buf = static_cast <GstBuffer *> (user_data);
    gst_buffer_unref (buf);
  }

  void
  Uridecodebin::pad_to_shmdata_writer (GstElement *bin, GstPad *pad)
  {
    //detecting type of media
    const gchar *padname;
    if (0 == g_strcmp0 ("ANY", gst_caps_to_string (gst_pad_get_caps (pad))))
      padname = "custom";
    else
      padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));

    g_debug ("uridecodebin new pad name is %s\n",padname);
    
    GstElement *fakesink;
    GstUtils::make_element ("fakesink", &fakesink);
    g_object_set (fakesink, 
     		  "sync", TRUE, 
          "signal-handoffs", TRUE,
     		  nullptr);
    GstElement *funnel;
    GstUtils::make_element ("funnel", &funnel);
    
    auto parent = gst_pad_get_parent(pad);
    auto grandparent = gst_object_get_parent(parent);
    gst_object_unref(grandparent);
    gst_object_unref(parent);
    gst_bin_add_many (GST_BIN (bin), fakesink, funnel, nullptr);
    GstUtils::link_static_to_request (pad, funnel);
    gst_element_link (funnel, fakesink);

    //GstUtils::wait_state_changed (bin);
    GstUtils::sync_state_with_parent (fakesink);
    GstUtils::sync_state_with_parent (funnel);
    
    //probing eos   
    GstPad *srcpad = gst_element_get_static_pad (funnel, "src");
    if (main_pad_ == nullptr)
      main_pad_ = srcpad;//saving first pad for looping
    gst_pad_add_event_probe (srcpad, (GCallback) event_probe_cb, this);   
    gst_object_unref (srcpad);

    //giving a name to the stream
    gchar **padname_splitted = g_strsplit_set (padname, "/",-1);
    //counting 
    int count = 0;
    auto it = media_counters_.find (padname_splitted[0]);
    if (media_counters_.end () != it)
      count = it->second + 1;
    //else
    //  {
	  //    std::string media_type ("unknown");
	  //    if (nullptr != padname_splitted[0])
	  //      media_type = padname_splitted[0];
	  //    media_counters_[media_type] = count;
    //  }
    media_counters_[padname_splitted[0]] = count;
	 
    gchar media_name[256];
    g_sprintf (media_name,"%s-%d",padname_splitted[0],count);
    g_debug ("uridecodebin: new media %s %d\n",media_name, count );
    g_strfreev(padname_splitted);

    ShmdataAnyWriter::ptr connector = std::make_shared<ShmdataAnyWriter>();
    std::string connector_name = make_file_name (media_name);
    connector->set_path (connector_name.c_str());

    g_signal_connect (fakesink, "handoff", (GCallback)on_handoff_cb, connector.get());

    g_message ("%s created a new shmdata any writer (%s)", 
    	       get_nick_name ().c_str(), 
    	       connector_name.c_str ());
    register_shmdata (connector);
  }

  gboolean 
  Uridecodebin::gstrtpdepay_buffer_probe_cb (GstPad */*pad*/, 
					     GstMiniObject */*mini_obj*/, 
					     gpointer user_data)
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
  Uridecodebin::gstrtpdepay_event_probe_cb (GstPad */*pad*/, 
					    GstEvent *event, 
					    gpointer user_data)
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
	gst_element_get_state (rtpgstdepay, nullptr, nullptr, GST_CLOCK_TIME_NONE);
	context->pad_to_shmdata_writer (context->bin_, srcpad);
	//gst_object_unref (srcpad);
      }
    else
      context->pad_to_shmdata_writer (context->bin_, pad);
  }   

  void 
  Uridecodebin::source_setup_cb (GstElement *uridecodebin, GstElement *source, gpointer user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
    g_debug ("uridecodebin source element is %s %s\n",  
	     GST_ELEMENT_NAME(source), 
	     G_OBJECT_CLASS_NAME (G_OBJECT_GET_CLASS (source)));

    //get the uri
    GValue val = G_VALUE_INIT;
    g_value_init (&val, G_TYPE_STRING);
    
    g_object_get_property (G_OBJECT (uridecodebin),
			   "uri",
			   &val);
    
    gchar *val_str = GstUtils::gvalue_serialize (&val);

    //building the command
    context->clean_on_error_command ();
    context->on_error_command_ = new QuiddityCommand ();
    context->on_error_command_->id_ = QuiddityCommand::set_property;
    context->on_error_command_->time_ = 1000; // 1 second
    context->on_error_command_->add_arg (context->get_nick_name ());
    context->on_error_command_->add_arg ("started");
    std::vector<std::string> vect_arg;
    vect_arg.push_back ("false");
    context->on_error_command_->set_vector_arg (vect_arg);

    g_object_set_data (G_OBJECT (source), 
     		       "on-error-command",
     		       (gpointer)context->on_error_command_);
    
    g_free (val_str);
  }

  bool
  Uridecodebin::to_shmdata ()
  {
    destroy_uridecodebin ();
    reset_bin ();
    init_uridecodebin ();
    g_debug ("to_shmdata set uri %s", uri_);
    g_object_set (G_OBJECT (uridecodebin_), "uri", uri_, nullptr); 
    gst_bin_add (GST_BIN (bin_), uridecodebin_);
    //GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (uridecodebin_);
    return true;
  }

  void 
  Uridecodebin::set_loop (gboolean loop, void *user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *> (user_data);
    context->loop_ = loop;
  }

  gboolean 
  Uridecodebin::get_loop (void *user_data)
  {
    Uridecodebin *context = static_cast<Uridecodebin *> (user_data);
    return context->loop_;
  }

  void 
  Uridecodebin::set_uri (const gchar *value, void *user_data)
  {
    Uridecodebin *context = static_cast <Uridecodebin *> (user_data);
    g_free (context->uri_);
    context->uri_ = g_strdup (value);
    context->to_shmdata ();
    context->query_position_and_length ();
    context->custom_props_->notify_property_changed (context->uri_spec_);
  }

  const gchar *
  Uridecodebin::get_uri (void *user_data)
  {
    Uridecodebin *context = static_cast <Uridecodebin *> (user_data);
    return context->uri_;
  }
}
