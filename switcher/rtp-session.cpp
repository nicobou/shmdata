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

#include "rtp-session.h"
#include <sstream>
#include <glib/gstdio.h> //writing sdp file
#include "gst-utils.h"
#include "json-builder.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(RtpSession,
				       "RTP Session", 
				       "network",
				       "RTP session manager",
				       "LGPL",
				       "rtpsession",
				       "Nicolas Bouillot");

  RtpSession::RtpSession () :
    rtpsession_ (nullptr),
    next_id_ (79), //this value is arbitrary and can be changed
    custom_props_ (new CustomPropertyHelper ()), 
    destination_description_json_ (nullptr),
    destinations_json_ (nullptr),
    mtu_at_add_data_stream_spec_ (nullptr),
    mtu_at_add_data_stream_ (1400),
    internal_id_ (),
    rtp_ids_ (),
    quiddity_managers_ (),
    funnels_ (),
    internal_shmdata_writers_ (),
    internal_shmdata_readers_ (),
    destinations_ ()
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
  }
  
  RtpSession::~RtpSession ()
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    
    std::vector <std::string> paths;
    for (auto &it : quiddity_managers_)
      paths.push_back (it.first);
    for (auto &it : paths)
	remove_data_stream (it);

    //removing sdp files
    std::vector <std::string> destinations;
    for (auto &it : destinations_)
      destinations.push_back (it.first);
    for (auto &it :destinations)
      {
	std::string sdp_file = make_file_name (it);
	sdp_file.append(".sdp");
	g_remove (sdp_file.c_str ());
      }

    // if (GST_IS_BIN (gst_element_get_parent (rtpsession_)))
    //   {
    // 	GstElement *parent = (GstElement *)gst_element_get_parent (rtpsession_);
    // 	g_debug ("%d, %d, %d, state return %d",
    // 		 GST_STATE(parent),
    // 		 GST_STATE_TARGET (parent), 
    // 		 GST_STATE_PENDING (parent),
    // 		 GST_STATE_RETURN(parent));
    //   }
    
    // //removing rtpsession
    // GstUtils::clean_element (rtpsession_);
    
    g_debug ("rtpsession deleted");
    }

  bool 
  RtpSession::init_gpipe ()
  {
    if (!GstUtils::make_element ("gstrtpbin", &rtpsession_))
      return false;
    g_object_set (G_OBJECT (rtpsession_), "ntp-sync", TRUE, nullptr);

    g_object_set (G_OBJECT (bin_), "async-handling", TRUE, nullptr);
    
    g_signal_connect (G_OBJECT (rtpsession_), "on-bye-ssrc", 
		      (GCallback) on_bye_ssrc, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-bye-timeout", 
		      (GCallback) on_bye_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-new-ssrc", 
		      (GCallback) on_new_ssrc, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-npt-stop", 
		      (GCallback)  on_npt_stop, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-sender-timeout",  
		      (GCallback) on_sender_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-ssrc-active",  
		      (GCallback)  on_ssrc_active, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-ssrc-collision",  
		      (GCallback) on_ssrc_collision, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-ssrc-sdes",  
		      (GCallback)  on_ssrc_sdes, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-ssrc-validated",  
		      (GCallback) on_ssrc_validated, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on-timeout",  
		      (GCallback) on_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "pad-added",  
		      (GCallback) on_pad_added, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "pad-removed",  
		      (GCallback) on_pad_removed, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "no-more-pads",  
		      (GCallback) on_no_more_pad, (gpointer) this);

    gst_bin_add (GST_BIN (bin_), rtpsession_);
    GstUtils::sync_state_with_parent (rtpsession_);

    install_method ("Add Data Stream",
		    "add_data_stream", 
		    "add a data stream to the RTP session (sending)", 
		    "succes or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "socket", 
						  "shmdata socket path to add to the session",
						  nullptr),
		    (Method::method_ptr) &add_data_stream_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);

    install_method ("Remove Data Stream",
		    "remove_data_stream", 
		    "remove a data stream from the RTP session (sending)", 
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "socket", 
						  "shmdata socket path to remove from the session", 
						  nullptr),
		    (Method::method_ptr) &remove_data_stream_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);

    install_method ("Add Destination",
		    "add_destination", 
		    "add a destination (two destinations can share the same host name)", 
		    "success or fail",
		    Method::make_arg_description ("Name",
						  "name", 
						  "a destination name (user defined)",
						  "Host Name or IP",
						  "host_name",
						  "the host name of the destination",
						  nullptr),
		    (Method::method_ptr) &add_destination_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, G_TYPE_STRING, nullptr),
		    this);
    
    install_method ("Remove Destination",
		    "remove_destination", 
		    "remove a destination",
		    "success or fail",
		    Method::make_arg_description ("Name",
						  "name", 
						  "the destination name",
						  nullptr),
		    (Method::method_ptr) &remove_destination_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);


    install_method ("Add UDP Stream",
		    "add_udp_stream_to_dest", 
		    "stream RTP to a port with udp", 
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path", "socket", "local socket path of the shmdata",
						  "Destination", "dest", "name of the destination",
						  "Port", "port", "destination port",
						  nullptr),
		    (Method::method_ptr) &add_udp_stream_to_dest_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING,nullptr),
		    this);
    
    install_method ("Remove UDP Stream",
		   "remove_udp_stream_to_dest", 
		    "remove destination", 
		    "succes or fail",
		    Method::make_arg_description ("Shmdata Path", "socket", "local socket path of the shmdata",
						  "Destination", "dest_name", "destination name",
						  nullptr),
      		    (Method::method_ptr) &remove_udp_stream_to_dest_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING,G_TYPE_STRING,nullptr),
		    this);
    
    install_method ("Write SDP File",
		    "write_sdp_file", 
		    "print sdp for the given destination", 
		    "success or fail",
		    Method::make_arg_description ("Destination",
						  "name", 
						  "the name of the destination",
						  nullptr),
		    (Method::method_ptr) &write_sdp_file_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING,nullptr),
		    this);

    destination_description_json_ = custom_props_->make_string_property ("destinations-json", 
									 "json formated description of destinations",
									 "",
									 (GParamFlags) G_PARAM_READABLE,
									 nullptr,
									 RtpSession::get_destinations_json,
									 this);

    install_property_by_pspec (custom_props_->get_gobject (), 
				destination_description_json_, 
				"destinations-json",
				"Destinations");

    mtu_at_add_data_stream_spec_ = custom_props_->make_int_property ("mtu-at-add-data-stream", 
								     "MTU that will be set during add_data_stream invokation",
								     0,
								     15000,
								     1400,
								     (GParamFlags) G_PARAM_READWRITE,
								     RtpSession::set_mtu_at_add_data_stream,
								     RtpSession::get_mtu_at_add_data_stream,
								     this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				mtu_at_add_data_stream_spec_, 
				"mtu-at-add-data-stream",
				"MTU At Add Data Stream");
    return true;
  }
  
  gboolean
  RtpSession::write_sdp_file_wrapped (gpointer nick_name, 
				 gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);
    
    if (context->write_sdp_file ((char *)nick_name))
      return TRUE;
    else
      return FALSE;
  }


  bool
  RtpSession::write_sdp_file (std::string dest_name)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    auto it = destinations_.find (dest_name);
    if (destinations_.end () == it)
      {
	g_warning ("RtpSession: destination named %s does not exists, cannot print sdp ",
		   dest_name.c_str ());
	return false;
      }
    std::string res = it->second->get_sdp();
    
    if (res == "")
      return false;

    std::string sdp_file = make_file_name (dest_name);
    sdp_file.append(".sdp");

    g_remove (sdp_file.c_str ());
    
    if (!g_file_set_contents (sdp_file.c_str (), 
			      res.c_str (), 
			      -1, //no size, res is a null terminated string
			      nullptr)) //not getting errors
      return false;
    
    return true;
  }

  //function used as a filter for selecting the appropriate rtp payloader
  gboolean
  RtpSession::sink_factory_filter (GstPluginFeature * feature, gpointer data)
  {
    ////g_print ("%s\n", __PRETTY_FUNCTION__);
    // guint rank;
    const gchar *klass;
    
    GstCaps* caps = (GstCaps *)data;

    // searching element factories only
    if (!GST_IS_ELEMENT_FACTORY (feature))
      return FALSE;
    
    klass = gst_element_factory_get_klass (GST_ELEMENT_FACTORY (feature));
    if (!(g_strrstr (klass, "Payloader") && g_strrstr (klass, "RTP")))
      return FALSE;
    
    if (!gst_element_factory_can_sink_caps (GST_ELEMENT_FACTORY (feature), caps))
      return FALSE;
    
    return TRUE;
  }
  
  //sorting factory by rank
  gint
  RtpSession::sink_compare_ranks (GstPluginFeature * f1, GstPluginFeature * f2)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    gint diff;
    
    diff = gst_plugin_feature_get_rank (f2) - gst_plugin_feature_get_rank (f1);
    if (diff != 0)
      return diff;
    return g_strcmp0 (gst_plugin_feature_get_name (f2),
		      gst_plugin_feature_get_name (f1));
  }

  //this is a typefind function, called when type of input stream from a shmdata is found
  void
  RtpSession::make_data_stream_available (GstElement* typefind, 
					  guint /*probability*/, 
					  GstCaps *caps, 
					  gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("RtpSession::make_data_stream_available");
    GstElement *pay = nullptr;
    GList *list = gst_registry_feature_filter (gst_registry_get_default (),
					       (GstPluginFeatureFilter) sink_factory_filter, 
					       FALSE, caps);
    list = g_list_sort (list, (GCompareFunc) sink_compare_ranks);

    //bypassing jpeg for high dimensions
    bool jpeg_payloader = true;
     GstStructure *caps_structure = gst_caps_get_structure (caps, 0);
     if (g_str_has_prefix (gst_structure_get_name (caps_structure), "image/jpeg"))
       {//check jpeg dimension are suported by jpeg payloader
     	gint width = 0, height = 0;
     	/* these properties are not mandatory, we can get them from the SOF, if there
     	 * is one. */
     	if (gst_structure_get_int (caps_structure, "height", &height)) {
     	  if (height <= 0 || height > 2040)
     	    jpeg_payloader = false;
     	}
     	if (gst_structure_get_int (caps_structure, "width", &width)) {
     	  if (width <= 0 || width > 2040)
     	    jpeg_payloader = false;
     	}
       }
      
    if (list != nullptr && jpeg_payloader)  
      pay = gst_element_factory_create (GST_ELEMENT_FACTORY (list->data), nullptr);
    else
      GstUtils::make_element ("rtpgstpay", &pay);
    
    ShmdataReader *reader = (ShmdataReader *) g_object_get_data (G_OBJECT (typefind),
								 "shmdata-reader");
    reader->add_element_to_cleaner (pay);
        
    g_debug ("RtpSession::make_data_stream_available: %s payloader",
	     GST_ELEMENT_NAME (pay));

    /* add capture and payloading to the pipeline and link */
    gst_bin_add_many (GST_BIN (context->bin_), pay, nullptr);
    gst_element_link (typefind, pay);
    g_debug ("%s sync",
	     __PRETTY_FUNCTION__);
    GstUtils::sync_state_with_parent (pay);
    g_debug ("%s after sync",
	     __PRETTY_FUNCTION__);
    g_object_set (G_OBJECT (pay), "mtu", (guint)context->mtu_at_add_data_stream_, nullptr);
    
    /* now link all to the rtpbin, start by getting an RTP sinkpad for session "%d" */
    GstPad *sinkpad = gst_element_get_request_pad (context->rtpsession_, 
						   "send_rtp_sink_%d");
    GstPad *srcpad = gst_element_get_static_pad (pay, 
						 "src");
    if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
      g_warning ("RtpSession::make_data_stream_available: failed to link payloader to rtpbin");
    gst_object_unref (sinkpad);
    gst_object_unref (srcpad);
    
    //get name for the newly created pad
    gchar *rtp_sink_pad_name = gst_pad_get_name (sinkpad);
    gchar **rtp_session_array = g_strsplit_set (rtp_sink_pad_name, "_",0);
    
    gchar *rtp_session_id = g_strdup(rtp_session_array[3]);
    context->rtp_ids_[reader->get_path ()] = rtp_session_id;
    //using internal session id for this local stream
    std::string internal_session_id (context->internal_id_[reader->get_path ()]);
    
    g_free (rtp_sink_pad_name);
    g_strfreev (rtp_session_array);
    
    // rtp src pad (is a static pad since created after the request of rtp sink pad)
    gchar *rtp_src_pad_name = g_strconcat ("send_rtp_src_", rtp_session_id, nullptr); 
    g_debug ("RtpSession: request rtp src pad and create a corresponding shmwriter %s",rtp_src_pad_name);
    GstPad *rtp_src_pad = gst_element_get_static_pad (context->rtpsession_, rtp_src_pad_name);

    if (!GST_IS_PAD (rtp_src_pad)) 
      g_warning ("RtpSession: rtp_src_pad is not a pad"); 

    // testing
    // GstElement *myfake;
    // //GstUtils::make_element ("fakesink", &myfake);
    // GstUtils::make_element ("multiudpsink", &myfake);
    // g_object_set (G_OBJECT (myfake), "sync", FALSE, nullptr);
    
    // gst_bin_add_many (GST_BIN (context->bin_), myfake, nullptr);
    // GstPad *fakepad = gst_element_get_static_pad (myfake, "sink");
    // if (gst_pad_link (rtp_src_pad, fakepad) != GST_PAD_LINK_OK)
    //   g_debug ("RtpSession::make_data_stream_available linking with multiudpsink failled");
    // GstUtils::sync_state_with_parent (myfake);
    
    // g_signal_emit_by_name (myfake, "add", "localhost", "8044", nullptr);
    // //g_signal_emit_by_name (myfake, "add", "localhost", "9040", nullptr);
    
    //end testing
    
     ShmdataWriter::ptr rtp_writer;
     rtp_writer.reset (new ShmdataWriter ());
     std::string rtp_writer_name = context->make_file_name ("send_rtp_src_"+internal_session_id); 
     rtp_writer->set_path (rtp_writer_name.c_str ());
     rtp_writer->plug (context->bin_, rtp_src_pad);
     context->internal_shmdata_writers_[rtp_writer_name] = rtp_writer;
     g_free (rtp_src_pad_name);
     //set call back for saving caps in a property tree
     rtp_writer->set_on_caps (std::bind (&RtpSession::on_rtp_caps, 
					 context,
					 reader->get_path (),//std::move (rtp_writer_name),
					 std::placeholders::_1));

     //rtcp src pad
     gchar *rtcp_src_pad_name = g_strconcat ("send_rtcp_src_", rtp_session_id,nullptr); 
     g_debug ("RtpSession: request rtcp src pad %s",rtcp_src_pad_name);
     GstPad *rtcp_src_pad = gst_element_get_request_pad (context->rtpsession_, rtcp_src_pad_name);
     ShmdataWriter::ptr rtcp_writer;
     rtcp_writer.reset (new ShmdataWriter ());
     std::string rtcp_writer_name = context->make_file_name ("send_rtcp_src_"+internal_session_id); 
     rtcp_writer->set_path (rtcp_writer_name.c_str());
     rtcp_writer->plug (context->bin_, rtcp_src_pad);
     context->internal_shmdata_writers_[rtcp_writer_name] = rtcp_writer;
     g_free (rtcp_src_pad_name);

     // We also want to receive RTCP, request an RTCP sinkpad for given session and
     // link it to a funnel for future linking with network connections
     GstElement *funnel;
     GstUtils::make_element ("funnel", &funnel);
     gst_bin_add (GST_BIN (context->bin_), funnel);
     //GstUtils::wait_state_changed (context->bin_);
     GstUtils::sync_state_with_parent (funnel);
     GstPad *funnel_src_pad = gst_element_get_static_pad (funnel, "src");
     gchar *rtcp_sink_pad_name = g_strconcat ("recv_rtcp_sink_", rtp_session_id,nullptr); 
     GstPad *rtcp_sink_pad = gst_element_get_request_pad (context->rtpsession_, rtcp_sink_pad_name);
     if (gst_pad_link (funnel_src_pad, rtcp_sink_pad) != GST_PAD_LINK_OK)
       g_warning ("RtpSession::make_data_stream_available: Failed to link rtcpsrc to rtpbin");
     gst_object_unref (funnel_src_pad);
     g_free (rtcp_sink_pad_name);
     //for cleaning of funnel
     GstElementCleaner::ptr funnel_cleaning;
     funnel_cleaning.reset (new GstElementCleaner ());
     funnel_cleaning->add_element_to_cleaner (funnel);
     //saving funnel for being retried
     funnel_cleaning->add_labeled_element_to_cleaner ("funnel",funnel);
     context->funnels_[reader->get_path ()] = funnel_cleaning;
     g_free (rtp_session_id);
    
    g_debug ("RtpSession::make_data_stream_available (done)");
  }
  
  void 
  RtpSession::attach_data_stream(ShmdataReader *caller, void *rtpsession_instance)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession *>(rtpsession_instance);
    GstElement *funnel, *typefind;
    GstUtils::make_element ("funnel",&funnel);
    GstUtils::make_element ("typefind",&typefind);
    //give caller to typefind in order to register telement to remove
    g_object_set_data (G_OBJECT (typefind), "shmdata-reader", (gpointer)caller);
    g_signal_connect (typefind, "have-type", G_CALLBACK (RtpSession::make_data_stream_available), context);
    gst_bin_add_many (GST_BIN (context->bin_), funnel, typefind, nullptr);
    gst_element_link (funnel, typefind);
    
    //GstUtils::wait_state_changed (context->bin_);
    GstUtils::sync_state_with_parent (funnel);
    GstUtils::sync_state_with_parent (typefind);
    caller->set_sink_element (funnel);
    caller->add_element_to_cleaner (funnel);
    caller->add_element_to_cleaner (typefind);
  }
  


  gboolean
  RtpSession::add_destination_wrapped (gpointer nick_name, 
				       gpointer host_name,
				       gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_destination ((char *)nick_name,(char *)host_name))
      return TRUE;
    else
      return FALSE;
  }
  
  bool 
  RtpSession::add_destination (std::string nick_name,std::string host_name)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    if (destinations_.end () != destinations_.find (nick_name))
      {
	g_warning ("RtpSession: a destination named %s already exists, cannot add",
		   nick_name.c_str ());
	return false;
      }
    RtpDestination::ptr dest;
    dest.reset (new RtpDestination ());
    dest->set_name (nick_name);
    dest->set_host_name (host_name);
    destinations_[nick_name] = dest;
    return true;
  }

  gboolean
  RtpSession::remove_destination_wrapped (gpointer nick_name, 
					  gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->remove_destination ((char *)nick_name))
      return TRUE;
    else
      return FALSE;
  }
  
  bool 
  RtpSession::remove_destination (std::string nick_name)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    auto it = destinations_.find (nick_name);
    if (destinations_.end () == it)
      {
	g_warning ("RtpSession: destination named %s does not exists, cannot remove",
		   nick_name.c_str ());
	return false;
      }
    destinations_.erase (it);
    return true;
  }

  gboolean
  RtpSession::add_udp_stream_to_dest_wrapped (gpointer shmdata_name, 
					      gpointer nick_name, 
					      gpointer port, 
					      gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_udp_stream_to_dest ((char *)shmdata_name,(char *)nick_name,(char *)port))
      return TRUE;
    else
      return FALSE;
  }
  
  bool
  RtpSession::add_udp_stream_to_dest (std::string shmdata_socket_path, std::string nick_name, std::string port)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    auto id_it = internal_id_.find (shmdata_socket_path);
    if (internal_id_.end () == id_it)
      {
	g_warning ("RtpSession is not connected to %s",shmdata_socket_path.c_str ());
	return false;
      }
    auto destination_it = destinations_.find (nick_name);
    if (destinations_.end () == destination_it)
      {
	g_warning ("RtpSession does not contain a destination named %s",nick_name.c_str ());
	return false;
      }
    if (g_strcmp0 (destination_it->second->get_port (shmdata_socket_path).c_str (),
		   port.c_str ()) == 0) 
      {
	g_warning ("RtpSession: destination (%s) is already streaming shmdata (%s) to port %s",
		   nick_name.c_str (),
		   shmdata_socket_path.c_str (),
		   port.c_str ());
	return false;
      }
    //TODO check port has not been set for this destination
    gint rtp_port = atoi(port.c_str());
    
    if (rtp_port %2 !=0)
      {
	g_warning ("RtpSession rtp destination port %s must be even, not odd",port.c_str ());
	return false;
      }
    std::string id = id_it->second;

    auto manager_it = quiddity_managers_.find (shmdata_socket_path);
    if (quiddity_managers_.end () == manager_it)
      {
	//creating an internal quiddity manager 
	QuiddityManager::ptr manager = QuiddityManager::make_manager ("manager_"+get_name()+"_"+id);
	quiddity_managers_[shmdata_socket_path] = manager;

	std::vector <std::string> arg;
	manager->create ("udpsink","udpsend_rtp");
	manager->create ("udpsink","udpsend_rtcp");
	
	arg.clear ();
	arg.push_back (make_file_name ("send_rtp_src_"+id));
	manager->invoke ("udpsend_rtp", "connect", nullptr, arg);

	arg.clear ();
	arg.push_back (make_file_name ("send_rtcp_src_"+id));
	manager->invoke ("udpsend_rtcp", "connect", nullptr, arg);
	//update manager_it with the new one
	manager_it = quiddity_managers_.find (shmdata_socket_path);
      }
    QuiddityManager::ptr manager = manager_it->second;
    if (!(bool) manager)
      {
	g_warning ("add_udp_stream_to_dest ...not a manager...");
	return false;
      }
    //rtp stream (sending)
    RtpDestination::ptr dest = destinations_[nick_name];
    dest->add_stream (shmdata_socket_path, manager,port);
    std::vector <std::string> arg;
    arg.push_back (dest->get_host_name ());
    arg.push_back (port);
    manager->invoke ("udpsend_rtp","add_client", nullptr, arg);
    //rtcp stream (sending)
    arg.clear ();
    arg.push_back (dest->get_host_name ());
    std::ostringstream rtcp_port;
    rtcp_port << rtp_port + 1;
    arg.push_back (rtcp_port.str());
    manager->invoke ("udpsend_rtcp", "add_client", nullptr, arg);
    //TODO rtcp receiving should be negociated 
    // GstElementCleaner::ptr funnel_cleaner = funnels_[shmdata_socket_path];
    //  GstElement *funnel = funnel_cleaner->get_labeled_element_from_cleaner ("funnel");
    //  GstElement *udpsrc;
    //  GstUtils::make_element ("udpsrc", &udpsrc);
    //  dest->add_element_to_cleaner (udpsrc);
    //  g_object_set (G_OBJECT (udpsrc), "port", rtp_port + 5, nullptr);
    //  gst_bin_add (GST_BIN (bin_), udpsrc);
    //  GstUtils::sync_state_with_parent (udpsrc);
    //  if (!gst_element_link (udpsrc, funnel))
    //    g_debug ("udpsrc and funnel link failled in rtp-session");
    return true;
  }

  
  gboolean 
  RtpSession::remove_udp_stream_to_dest_wrapped (gpointer shmdata_socket_path, 
						 gpointer dest_name, 
						 gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);

    if (context->remove_udp_stream_to_dest ((char *)shmdata_socket_path, (char *)dest_name))
      return TRUE;
    else
      return FALSE;
  }
  
  bool
  RtpSession::remove_udp_stream_to_dest (std::string shmdata_socket_path, std::string dest_name)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    if (internal_id_.end () == internal_id_.find (shmdata_socket_path))
      {
	g_warning ("RtpSession is not connected to %s",shmdata_socket_path.c_str ());
	return false;
      }
    
    RtpDestination::ptr dest = destinations_[dest_name];
    if (!(bool) dest)
      {
	g_warning ("RtpSession::remove_udp_stream_to_dest, dest %s does not exist", dest_name.c_str ());
	return false;
      }
    std::string port = dest->get_port (shmdata_socket_path);
    dest->remove_stream (shmdata_socket_path);
    
    auto manager_it = quiddity_managers_.find (shmdata_socket_path);
    if (manager_it == quiddity_managers_.end ())
      {
	g_warning ("RtpSession::remove_udp_stream_to_dest, shmdata %s is not managed by the session", 
		   shmdata_socket_path.c_str ());
	return false;
      }
    
    QuiddityManager::ptr manager = manager_it->second;

    //rtp
    std::vector <std::string> arg;
    arg.push_back (dest->get_host_name ());
    arg.push_back (port);
    manager->invoke ("udpsend_rtp", "remove_client", nullptr, arg);
    
    //rtcp
    arg.clear ();
    gint rtp_port = atoi(port.c_str());
    arg.push_back (dest->get_host_name ());
    std::ostringstream rtcp_port;
    rtcp_port << rtp_port + 1;
    arg.push_back (rtcp_port.str());
    manager->invoke ("udpsend_rtp", "remove_client", nullptr, arg);
    return true;
  }

  gboolean
  RtpSession::add_data_stream_wrapped (gpointer connector_name, gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_data_stream ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  RtpSession::add_data_stream (std::string shmdata_socket_path)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    // if (internal_id_.end () != internal_id_.find (shmdata_socket_path))
    //   {
    // 	g_warning ("RtpSession::add_data_stream : stream not added since already managed");
    // 	return false;
    //   }
    remove_data_stream (shmdata_socket_path);
    ShmdataReader::ptr reader;
    reader.reset (new ShmdataReader ());
    reader->set_path (shmdata_socket_path.c_str());
    reader->set_g_main_context (get_g_main_context ());
    reader->set_bin (bin_);
    reader->set_on_first_data_hook (attach_data_stream, this);
    reader->start ();
    //saving info about this local stream
    std::ostringstream os_id;
    os_id << next_id_;
    next_id_++;
    internal_id_[shmdata_socket_path] = os_id.str();
    register_shmdata (reader);
    return true;
  }  

  gboolean
  RtpSession::remove_data_stream_wrapped (gpointer connector_name, gpointer user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession*>(user_data);
    if (context->remove_data_stream ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  RtpSession::remove_data_stream (std::string shmdata_socket_path)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    auto internal_id_it = internal_id_.find (shmdata_socket_path);
    if (internal_id_.end () == internal_id_it)
      {
	g_debug ("RtpSession::remove_data_stream: %s not present",
		 shmdata_socket_path.c_str ());
	return false;
      }
    quiddity_managers_.erase (shmdata_socket_path);
    for (auto &it : destinations_)
      {
	if (it.second->has_shmdata (shmdata_socket_path))
	  it.second->remove_stream (shmdata_socket_path);
      }
    std::string id = internal_id_it->second;
    internal_id_.erase (internal_id_it);
    auto writer_it = internal_shmdata_writers_.find (make_file_name ("send_rtp_src_"+id));
    if (internal_shmdata_writers_.end () != writer_it)
      {
	internal_shmdata_writers_.erase (writer_it);
	prune_tree ("rtp_caps." + make_file_name ("send_rtp_src_"+id));
      }
    writer_it = internal_shmdata_writers_.find (make_file_name ("send_rtcp_src_"+id));
    if (internal_shmdata_writers_.end () != writer_it)
      internal_shmdata_writers_.erase (writer_it);
    unregister_shmdata (shmdata_socket_path);
    auto reader_it = internal_shmdata_readers_.find (make_file_name ("recv_rtcp_sink_"+id));
    if (internal_shmdata_readers_.end () != reader_it)
      internal_shmdata_readers_.erase (reader_it);
    auto funnel_it = funnels_.find (shmdata_socket_path);
    if (funnels_.end () == funnel_it)
      {
	g_debug ("RtpSession::remove_data_stream: no funnel");
	return false;
      }
    funnels_.erase (funnel_it);
    rtp_ids_.erase (shmdata_socket_path);
    g_debug ("data_stream %s removed", shmdata_socket_path.c_str ());
    return true;
  }

  void
  RtpSession::on_bye_ssrc (GstElement */*rtpbin*/, 
			   guint /*session*/, 
			   guint /*ssrc*/, 
			   gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_bye_ssrc");
  }

  void
  RtpSession::on_bye_timeout (GstElement */*rtpbin*/, 
			      guint /*session*/, 
			      guint /*ssrc*/, 
			      gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_bye_timeout");
  }

  void
  RtpSession::on_new_ssrc (GstElement */*rtpbin*/, 
			   guint /*session*/, 
			   guint /*ssrc*/, 
			   gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_new_ssrc");
  }

  void
  RtpSession::on_npt_stop (GstElement */*rtpbin*/, 
			   guint /*session*/, 
			   guint /*ssrc*/, 
			   gpointer /*user_data*/)
  {
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    g_debug ("on_npt_stop");
  }

  void
  RtpSession::on_sender_timeout (GstElement */*rtpbin*/, 
				 guint /*session*/, 
				 guint /*ssrc*/, 
				 gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_sender_timeout");
  }

  void
  RtpSession::on_ssrc_active (GstElement */*rtpbin*/, 
			      guint /*session*/, 
			      guint /*ssrc*/, 
			      gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_ssrc_active");
  }

  void
  RtpSession::on_ssrc_collision (GstElement */*rtpbin*/, 
				 guint /*session*/, 
				 guint /*ssrc*/, 
				 gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_ssrc_active");
  }

  void
  RtpSession::on_ssrc_sdes (GstElement */*rtpbin*/, 
			    guint /*session*/, 
			    guint /*ssrc*/, 
			    gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_ssrc_sdes");
  }

  void
  RtpSession::on_ssrc_validated (GstElement */*rtpbin*/, 
				 guint /*session*/, 
				 guint /*ssrc*/, 
				 gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_ssrc_validated");
  }
 
  void
  RtpSession::on_timeout (GstElement */*rtpbin*/, 
			  guint /*session*/, 
			  guint /*ssrc*/, 
			  gpointer /*user_data*/)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_timeout");
  }

  void
  RtpSession::on_pad_added (GstElement */*gstelement*/, 
			    GstPad *new_pad, 
			    gpointer user_data) 
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_pad_added, name: %s, direction: %d", 
	     gst_pad_get_name(new_pad),
	     gst_pad_get_direction (new_pad));
    //gchar *bidule = g_strdup ("bidule");
    context->signal_emit ("truc", "bidule");
  }

  void
  RtpSession::on_pad_removed (GstElement */*gstelement*/, 
			      GstPad *new_pad, 
			      gpointer /*user_data*/) 
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_pad_removed, name: %s, direction: %d", 
	     gst_pad_get_name(new_pad),
	     gst_pad_get_direction (new_pad));
    
  }

  void
  RtpSession::on_no_more_pad (GstElement */*gstelement*/, 
			      gpointer /*user_data*/) 
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    //RtpSession *context = static_cast<RtpSession *>(user_data);
    g_debug ("on_no_more_pad");
  }

  const gchar *
  RtpSession::get_destinations_json (void *user_data)
  {
    //g_print ("%s\n", __PRETTY_FUNCTION__);
    RtpSession *context = static_cast<RtpSession *> (user_data);

    if (context->destinations_json_ != nullptr)
      g_free (context->destinations_json_);

    JSONBuilder::ptr destinations_json (new JSONBuilder ());
    destinations_json->reset();
    destinations_json->begin_object ();
    destinations_json->set_member_name ("destinations");
    destinations_json->begin_array ();
    for (auto &it : context->destinations_)
      destinations_json->add_node_value ( it.second->get_json_root_node ());
    destinations_json->end_array ();
    destinations_json->end_object ();
    context->destinations_json_ = g_strdup (destinations_json->get_string (true).c_str ());
    return context->destinations_json_;
  }

  void 
  RtpSession::set_mtu_at_add_data_stream (const gint value, void *user_data)
  {
    RtpSession *context = static_cast<RtpSession *> (user_data);
    context->mtu_at_add_data_stream_ = value;
  }
   
  gint RtpSession::get_mtu_at_add_data_stream (void *user_data)
  {
    RtpSession *context = static_cast<RtpSession *> (user_data);
    return context->mtu_at_add_data_stream_;
  }

  void
  RtpSession::on_rtp_caps (std::string shmdata_path, std::string caps)
  {
    graft_tree ("rtp_caps." + std::move (shmdata_path), 
		data::make_tree (std::move (caps)));
  }
}
