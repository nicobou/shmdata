/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/rtpsession.h"

namespace switcher
{
  
  QuiddityDocumentation RtpSession::doc_ ("RTP session", "rtpsession",
					 "RTP session manager");
  
  RtpSession::RtpSession ()
  {
    make_rtpsession ();
  }

  RtpSession::RtpSession (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_rtpsession ();
  }

  void 
  RtpSession::make_rtpsession ()
  {
    rtpsession_ = gst_element_factory_make ("gstrtpbin",NULL);
    
    g_signal_connect (G_OBJECT (rtpsession_), "on-bye-ssrc", 
		      (GCallback) on_bye_ssrc, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_bye_timeout", 
		      (GCallback) on_bye_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_new_ssrc", 
		      (GCallback) on_new_ssrc, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_npt_stop", 
		      (GCallback)  on_npt_stop, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_sender_timeout",  
		      (GCallback) on_sender_timeout, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_active",  
		      (GCallback)  on_ssrc_active, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_collision",  
		      (GCallback) on_ssrc_collision, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_sdes",  
		      (GCallback)  on_ssrc_sdes, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_ssrc_validated",  
		      (GCallback) on_ssrc_validated, (gpointer) this);
    g_signal_connect (G_OBJECT (rtpsession_), "on_timeout ",  
		      (GCallback) on_timeout, (gpointer) this);
    
    gst_bin_add (GST_BIN (bin_), rtpsession_);

    //registering add_data_stream
    std::vector<GType> add_data_stream_arg_types;
    add_data_stream_arg_types.push_back (G_TYPE_STRING);
    register_method("add_data_stream",(void *)&add_data_stream_wrapped, add_data_stream_arg_types,(gpointer)this);
    std::vector<std::pair<std::string,std::string> > arg_desc;
    std::pair<std::string,std::string> socket;
    socket.first = "socket";
    socket.second = "socket path of the shmdata to add to the session (sending)";
    arg_desc.push_back (socket); 
    if (!set_method_description ("add_data_stream", "add a data stream to the RTP session (sending)", arg_desc))
      g_printerr ("RTP session: cannot set method description for \"add_data_stream\"\n");

    //set the name before registering properties
    name_ = gst_element_get_name (rtpsession_);
    
    make_sdp_init ();
    
  }

  void
  RtpSession::make_sdp_init ()
  {
    //prepare SDP description
    gst_sdp_message_new (&sdp_description_);
    
    /* some standard things first */
    gst_sdp_message_set_version (sdp_description_, "0");
    
    //FIXME check and chose between IP4 and IP6, IP4 hardcoded
    gst_sdp_message_set_origin (sdp_description_, 
				"-",                // the user name
				"1188340656180883", // a session id
				"1",                // a session version
				"IN",               // a network type
				"IP4",              // an address type
				"localhost"); //an address
    username :
	the user name

sess_id :
	a session id

sess_version :
	a session version

nettype :
	a network type

addrtype :
	an address type

addr :
	an address

    gst_sdp_message_set_session_name (sdp_description_, "switcher session");
    gst_sdp_message_set_information (sdp_description_, "telepresence");
    gst_sdp_message_add_time (sdp_description_, "0", "0", NULL);
    gst_sdp_message_add_attribute (sdp_description_, "tool", "switcher (with GStreamer)");
    gst_sdp_message_add_attribute (sdp_description_, "type", "broadcast");
    gst_sdp_message_add_attribute (sdp_description_, "control", "*");

  }

  QuiddityDocumentation 
  RtpSession::get_documentation ()
  {
    return doc_;
  }

  //fuction used as a filter for selecting the appropriate rtp payloader
  gboolean
  RtpSession::sink_factory_filter (GstPluginFeature * feature, gpointer data)
  {
    // guint rank;
    const gchar *klass;
    
    GstCaps* caps = (GstCaps *)data;

    /* we only care about element factories */
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
    gint diff;
    
    diff = gst_plugin_feature_get_rank (f2) - gst_plugin_feature_get_rank (f1);
    if (diff != 0)
      return diff;
    return g_strcmp0 (gst_plugin_feature_get_name (f2),
		      gst_plugin_feature_get_name (f1));
  }

  //this is a typefind function, called when type of input stream from a shmdata is found
  void
  RtpSession::make_sender (GstElement* typefind, guint probability, GstCaps* caps, gpointer user_data)
  {
    RtpSession *context = static_cast<RtpSession *>(user_data);
    
    GstElement *pay;
    GstElement *rtpsink, *rtcpsink, *rtcpsrc;
    GstPad *srcpad, *sinkpad;

        
    GList *list = gst_registry_feature_filter (gst_registry_get_default (),
					       (GstPluginFeatureFilter) sink_factory_filter, 
					       FALSE, caps);
    list = g_list_sort (list, (GCompareFunc) sink_compare_ranks);

    if (list != NULL)
      pay = gst_element_factory_create (GST_ELEMENT_FACTORY (list->data), NULL);
    else
      pay = gst_element_factory_make ("rtpgstpay", NULL);

    g_print ("RtpSession: selection %s payloader\n",GST_ELEMENT_NAME (pay));

    /* add capture and payloading to the pipeline and link */
    gst_bin_add_many (GST_BIN (context->bin_), pay, NULL);
    gst_element_link (typefind, pay);
    gst_element_sync_state_with_parent (pay);

    /* the udp sinks and source we will use for RTP and RTCP */
    rtpsink = gst_element_factory_make ("udpsink", NULL);
    g_object_set (rtpsink, "port", 10000, "host", "localhost", NULL);

    rtcpsink = gst_element_factory_make ("udpsink", NULL);
    g_object_set (rtcpsink, "port", 10001, "host", "localhost", NULL);
   /* no need for synchronisation or preroll on the RTCP sink */
   g_object_set (rtcpsink, "async", FALSE, "sync", FALSE, NULL);

   rtcpsrc = gst_element_factory_make ("udpsrc", NULL);
   g_object_set (rtcpsrc, "port", 10005, NULL);

   gst_bin_add_many (GST_BIN (context->bin_), rtpsink, rtcpsink, rtcpsrc, NULL);

   /* now link all to the rtpbin, start by getting an RTP sinkpad for session 0 */
   sinkpad = gst_element_get_request_pad (context->rtpsession_, "send_rtp_sink_0");
   srcpad = gst_element_get_static_pad (pay, "src");
   if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
     g_error ("Failed to link payloader to rtpbin");
   gst_object_unref (srcpad);

   /* get the RTP srcpad that was created when we requested the sinkpad above and
    * link it to the rtpsink sinkpad*/
   srcpad = gst_element_get_static_pad (context->rtpsession_, "send_rtp_src_0");
   sinkpad = gst_element_get_static_pad (rtpsink, "sink");
   if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
     g_error ("Failed to link rtpbin to rtpsink");
   gst_object_unref (srcpad);
   gst_object_unref (sinkpad);

   /* get an RTCP srcpad for sending RTCP to the receiver */
   srcpad = gst_element_get_request_pad (context->rtpsession_, "send_rtcp_src_0");
   sinkpad = gst_element_get_static_pad (rtcpsink, "sink");
   if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
     g_error ("Failed to link rtpbin to rtcpsink");
   gst_object_unref (sinkpad);

   /* we also want to receive RTCP, request an RTCP sinkpad for session 0 and
    * link it to the srcpad of the udpsrc for RTCP */
   srcpad = gst_element_get_static_pad (rtcpsrc, "src");
   sinkpad = gst_element_get_request_pad (context->rtpsession_, "recv_rtcp_sink_0");
   if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
     g_error ("Failed to link rtcpsrc to rtpbin");
   gst_object_unref (srcpad);

   gst_element_sync_state_with_parent (rtpsink);
   gst_element_sync_state_with_parent (rtcpsink);
   gst_element_sync_state_with_parent (rtcpsrc);

  }

  void 
  RtpSession::attach_data_stream(ShmdataReader *caller, void *rtpsession_instance)
  {
    RtpSession *context = static_cast<RtpSession *>(rtpsession_instance);

    GstElement *funnel, *typefind;
    funnel = gst_element_factory_make ("funnel", NULL);
    typefind = gst_element_factory_make ("typefind", NULL);
    g_signal_connect (typefind, "have-type", G_CALLBACK (RtpSession::make_sender), context);
    gst_bin_add_many (GST_BIN (context->bin_), funnel, typefind, NULL);
    gst_element_link (funnel, typefind);
    gst_element_sync_state_with_parent (funnel);
    gst_element_sync_state_with_parent (typefind);

    caller->set_sink_element (funnel);

  }

   gboolean
   RtpSession::add_data_stream_wrapped (gpointer connector_name, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    RtpSession *context = static_cast<RtpSession*>(user_data);
       
    if (context->add_data_stream ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  

  bool
  RtpSession::add_data_stream (std::string shmdata_socket_path)
  {
    ShmdataReader::ptr reader;
    reader.reset (new ShmdataReader ());
    reader->set_path (shmdata_socket_path.c_str());
    reader->set_bin (bin_);
    //    reader_->set_sink_element (sink_element_);
    //if (connection_hook_ != NULL) 
    reader->set_on_first_data_hook (attach_data_stream, this);
    if (runtime_ != NULL) // starting the reader if runtime is set
      reader->start ();
    shmdata_readers_.insert (shmdata_socket_path, reader);
    return true;
  }  

  void
  RtpSession::on_bye_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_bye_ssrc\n");
  }

  void
  RtpSession::on_bye_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_bye_timeout\n");
  }

  void
  RtpSession::on_new_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_new_ssrc\n");
  }

  void
  RtpSession::on_npt_stop (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_npt_stop\n");
  }

  void
  RtpSession::on_sender_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_sender_timeout\n");
  }

  void
  RtpSession::on_ssrc_active  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_active\n");
  }

  void
  RtpSession::on_ssrc_collision (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_active\n");
  }

  void
  RtpSession::on_ssrc_sdes  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_sdes\n");
  }

  void
  RtpSession::on_ssrc_validated (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_validated\n");
  }
 
  void
  RtpSession::on_timeout  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_timeout\n");
  }


}
