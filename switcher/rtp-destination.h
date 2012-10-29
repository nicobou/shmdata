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


#ifndef __SWITCHER_RTPDESTINATION_H__
#define __SWITCHER_RTPDESTINATION_H__

#include <gst/sdp/gstsdpmessage.h>
#include <gst/gst.h>
#include "switcher/string-map.h"
#include "switcher/shmdata-reader.h"

namespace switcher
{
  class RtpDestination
  {
  public:
    typedef std::tr1::shared_ptr<RtpDestination> ptr;
    void set_host_name (std::string host_name);
    std::string get_host_name ();
    //the reader of the rtp stream sent
    bool add_stream (ShmdataReader::ptr rtp_shmdata_reader,std::string port);
    bool remove_stream (std::string shmdata_stream_path);
    std::string get_sdp ();

  private:
    std::string host_name_;
    StringMap<ShmdataReader::ptr> ports_; //maps port with thr rtp shmdata reader
    StringMap<std::string> source_streams_; //maps shmdata source stream with port
  };
}  // end of namespace

#endif // ifndef
