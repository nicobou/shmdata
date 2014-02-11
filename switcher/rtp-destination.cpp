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

#include "rtp-destination.h"
#include <sstream>

namespace switcher
{
  RtpDestination::RtpDestination ()
  {
    json_description_.reset (new JSONBuilder());
  }
  
  RtpDestination::~RtpDestination ()
  {
    for (auto &it : ports_)
      {
	QuiddityManager::ptr manager = it.second;
	//cleaning rtp
	std::vector <std::string> arg;
	arg.push_back (host_name_);
	arg.push_back (it.first);
	manager->invoke ("udpsend_rtp", "remove_client", NULL ,arg);
	//cleaning rtcp
	arg.clear ();
	arg.push_back (host_name_);
	std::ostringstream rtcp_port;
	rtcp_port << atoi(it.first.c_str()) + 1;
	arg.push_back (rtcp_port.str());
	manager->invoke ("udpsend_rtp", "remove_client", NULL, arg);
	//TODO remove connection to funnel
      }
  }
  
  void 
  RtpDestination::set_name (std::string name)
  {
    name_ = name;
    make_json_description ();
  }

  void 
  RtpDestination::set_host_name (std::string host_name)
  {
    host_name_ = host_name;
    make_json_description ();
  }
  
  std::string
  RtpDestination::get_host_name ()
  {
    return host_name_;
  }

  std::string 
  RtpDestination::get_port (std::string shmdata_path)
  {
    auto it = source_streams_.find (shmdata_path);
    if (source_streams_.end () == it)
      return "";
    return it->second;
  }


  bool 
  RtpDestination::add_stream (std::string orig_shmdata_path,
			      QuiddityManager::ptr manager, 
			      std::string port)
  {
    ports_[port] = manager;
    source_streams_[orig_shmdata_path] = port;
    make_json_description ();
    return true;
  }

  bool
  RtpDestination::has_shmdata (std::string shmdata_path)
  {
    return (source_streams_.end () != source_streams_.find (shmdata_path));
  }

  bool
  RtpDestination::has_port (std::string port)
  {
    return (ports_.end () != ports_.find (port));
  }
  
  bool
  RtpDestination::remove_stream (std::string shmdata_stream_path)
  {
    auto it = source_streams_.find (shmdata_stream_path);
    if (source_streams_.end () == it)
      {
	g_warning ("RtpDestination: stream not found, cannot remove %s", 
		   shmdata_stream_path.c_str ());
	return false;
      }
    ports_.erase (it->second);
    source_streams_.erase (it);
    make_json_description ();
    return true;
  }
  
  std::string 
  RtpDestination::get_sdp ()
  {
    GstSDPMessage *sdp_description;
    //prepare SDP description
    gst_sdp_message_new (&sdp_description);
    
    /* some standard things first */
    gst_sdp_message_set_version (sdp_description, "0");
    
    //FIXME check and chose between IP4 and IP6, IP4 hardcoded
    gst_sdp_message_set_origin (sdp_description, 
      				"-",                // the user name
      				"1188340656180883", // a session id
      				"1",                // a session version
      				"IN",               // a network type
      				"IP4",              // an address type
      				"localhost"); //an address

    gst_sdp_message_set_session_name (sdp_description, "switcher session");
    gst_sdp_message_set_information (sdp_description, "telepresence");
    gst_sdp_message_add_time (sdp_description, "0", "0", NULL);
    gst_sdp_message_add_attribute (sdp_description, "tool", "switcher");
    gst_sdp_message_add_attribute (sdp_description, "type", "broadcast");
    gst_sdp_message_add_attribute (sdp_description, "control", "*");

    gint index = 0;
    for(auto &it : ports_)
      {
	std::string string_caps = (it.second)->get_property ("udpsend_rtp","caps");
	if (g_strcmp0 (string_caps.c_str (), "NULL") != 0)
	  {
	    GstCaps *caps = gst_caps_from_string (string_caps.c_str ());

	    gint port = atoi(it.first.c_str());

	    sdp_write_media_from_caps (sdp_description, 
				       caps,
				       port,
				       "127.0.0.1",
				       "udp",
				       index);
	    index ++;
	    gst_caps_unref (caps);
	  }
	else
	  {
	    g_debug ("generating sdp file, empty media description %s (returning empty file)", string_caps.c_str ());
	    return "";
	  }

      }

    std::string res (gst_sdp_message_as_text (sdp_description));
    gst_sdp_message_free (sdp_description);
    
    return res;
  }


void
RtpDestination::sdp_write_media_from_caps (GstSDPMessage *sdp_description, 
					   GstCaps *media_caps,
					   gint dest_port,
					   std::string server_host_name,
					   std::string transport_proto,
					   gint stream_number)
{

  //TODO check if sdp "range" is useful/...
  
  GstSDPMedia *smedia;
  GstStructure *s;
  const gchar *caps_str, *caps_enc, *caps_params;
  gchar *tmp;
  gint caps_pt, caps_rate;
  guint n_fields, j;
  gboolean first;
  GString *fmtp;

    s = gst_caps_get_structure (media_caps, 0);
     
    gst_sdp_media_new (&smedia);
      
    /* get media type and payload for the m= line */
    caps_str = gst_structure_get_string (s, "media");
    gst_sdp_media_set_media (smedia, caps_str);

    gst_structure_get_int (s, "payload", &caps_pt);
    tmp = g_strdup_printf ("%d", caps_pt);
    gst_sdp_media_add_format (smedia, tmp);
    g_free (tmp);

    gst_sdp_media_set_port_info (smedia, dest_port, 1);
    gst_sdp_media_set_proto (smedia, "RTP/AVP");

    /* for the c= line */
    gst_sdp_media_add_connection (smedia, "IN", transport_proto.c_str (),
				  server_host_name.c_str (),
				  16, 0);

    /* get clock-rate, media type and params for the rtpmap attribute */
    gst_structure_get_int (s, "clock-rate", &caps_rate);
    caps_enc = gst_structure_get_string (s, "encoding-name");
    caps_params = gst_structure_get_string (s, "encoding-params");

    if (caps_enc) {
      if (caps_params)
	tmp = g_strdup_printf ("%d %s/%d/%s", caps_pt, caps_enc, caps_rate,
			       caps_params);
      else
	tmp = g_strdup_printf ("%d %s/%d", caps_pt, caps_enc, caps_rate);

      gst_sdp_media_add_attribute (smedia, "rtpmap", tmp);
      g_free (tmp);
    }

    /* the config uri */
    tmp = g_strdup_printf ("stream=%d", stream_number);
    gst_sdp_media_add_attribute (smedia, "control", tmp);
    g_free (tmp);

    /* collect all other properties and add them to fmtp */
    fmtp = g_string_new ("");
    g_string_append_printf (fmtp, "%d ", caps_pt);
    first = TRUE;
    n_fields = gst_structure_n_fields (s);
    for (j = 0; j < n_fields; j++) {
      const gchar *fname, *fval;

      fname = gst_structure_nth_field_name (s, j);
      
      /* filter out standard properties */
      if (g_strcmp0 (fname, "media") == 0)
	continue;
      if (g_strcmp0 (fname, "payload") == 0)
	continue;
      if (g_strcmp0 (fname, "clock-rate") == 0)
	continue;
      if (g_strcmp0 (fname, "encoding-name") == 0)
	continue;
      if (g_strcmp0 (fname, "encoding-params") == 0)
	continue;
      if (g_strcmp0 (fname, "ssrc") == 0)
	continue;
      if (g_strcmp0 (fname, "clock-base") == 0)
	continue;
      if (g_strcmp0 (fname, "seqnum-base") == 0)
	continue;

      if ((fval = gst_structure_get_string (s, fname))) {
	g_string_append_printf (fmtp, "%s%s=%s", first ? "" : ";", fname, fval);
	first = FALSE;
      }
    }
    if (!first) {
      tmp = g_string_free (fmtp, FALSE);
      gst_sdp_media_add_attribute (smedia, "fmtp", tmp);
      g_free (tmp);
    } else {
      g_string_free (fmtp, TRUE);
    }
    gst_sdp_message_add_media (sdp_description, smedia);
    gst_sdp_media_free (smedia);

}

  void
  RtpDestination::make_json_description ()
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("name", name_.c_str ());
    json_description_->add_string_member ("host_name", host_name_.c_str ());
    json_description_->set_member_name ("data_streams");
    json_description_->begin_array ();
    for (auto &it : source_streams_)
      {
	json_description_->begin_object ();
	json_description_->add_string_member ("path", it.first.c_str ());
	json_description_->add_string_member ("port", it.second.c_str ());
	json_description_->end_object ();
      }
    json_description_->end_array ();
    json_description_->end_object ();
  }

  JSONBuilder::Node 
  RtpDestination::get_json_root_node ()
  {
    return json_description_->get_root ();
  }


}
