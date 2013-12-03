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


#ifndef __SWITCHER_RTPDESTINATION_H__
#define __SWITCHER_RTPDESTINATION_H__

#include "shmdata-reader.h"
#include "quiddity-manager.h"
#include "json-builder.h"
#include <gst/sdp/gstsdpmessage.h>
#include <gst/gst.h>
#include <map>
#include <string>

namespace switcher
{
  class RtpDestination : public GstElementCleaner
  {
  public:
    typedef std::shared_ptr<RtpDestination> ptr;
    RtpDestination ();
    ~RtpDestination ();

    void set_name (std::string name);
    void set_host_name (std::string host_name);
    std::string get_host_name ();
    std::string get_port (std::string shmndata_path);
    //the reader of the rtp stream sent
    bool add_stream (std::string orig_shmdata_path,
		     QuiddityManager::ptr manager, 
		     std::string port);
    bool has_shmdata (std::string shmdata_path);
    bool has_port (std::string port);
    bool remove_stream (std::string shmdata_stream_path);
    std::string get_sdp ();
    //get json doc:
    JSONBuilder::Node get_json_root_node ();

  private:
    std::string name_;
    std::string host_name_;
    std::map<std::string, QuiddityManager::ptr> ports_; //maps port with rtp shmdata reader
    std::map<std::string, std::string> source_streams_; //maps shmdata source stream with port
    static void sdp_write_media_from_caps (GstSDPMessage *sdp_description, 
					   GstCaps *media_caps,
					   gint dest_port,
					   std::string server_host_name,
					   std::string transport_proto,
					   gint stream_number);
    JSONBuilder::ptr json_description_;
    void make_json_description ();
  };
}  // end of namespace

#endif // ifndef
