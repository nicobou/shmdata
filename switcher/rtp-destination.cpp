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
#include "sdp-utils.h"

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
	manager->invoke ("udpsend_rtp", "remove_client", nullptr ,arg);
	//cleaning rtcp
	arg.clear ();
	arg.push_back (host_name_);
	std::ostringstream rtcp_port;
	rtcp_port << atoi(it.first.c_str()) + 1;
	arg.push_back (rtcp_port.str());
	manager->invoke ("udpsend_rtp", "remove_client", nullptr, arg);
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
    SDPDescription desc;
    
    for(auto &it : ports_)
      {
	std::string string_caps = (it.second)->get_property ("udpsend_rtp","caps");
	GstCaps *caps = gst_caps_from_string (string_caps.c_str ());
	gint port = atoi(it.first.c_str());
	SDPMedia media;
	media.set_media_info_from_caps (caps);
	media.set_port (port);

	if (!desc.add_media (media))
	  g_warning ("a media has not been added to the SDP description");

	gst_caps_unref (caps);
      }

    
    return desc.get_string ();
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
