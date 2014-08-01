/*
 *
 * posture is free software; you can redistribute it and/or
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

#include "posture_display.h"

#include <iostream>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureDisplay,
				       "Point clouds display",
				       "video sink", 
				       "Display point clouds in a window",
				       "LGPL",
				       "posturedisplay",				
				       "Emmanuel Durand");

  PostureDisplay::PostureDisplay() :
    custom_props_(std::make_shared<CustomPropertyHelper> ())
  {
  }

  PostureDisplay::~PostureDisplay()
  {
  }

  bool
  PostureDisplay::init()
  {
    init_segment (this);

    install_connect_method (std::bind (&PostureDisplay::connect,
				       this, 
				       std::placeholders::_1),
			    nullptr, //no disconnect
			    std::bind (&PostureDisplay::disconnect_all,
				       this),
			    std::bind (&PostureDisplay::can_sink_caps,
				       this, 
				       std::placeholders::_1),
			    1);

    return true;
  }

  bool
  PostureDisplay::connect (std::string shmdata_socket_path)
  {
    display_ = make_shared<Display>(shmdata_socket_path);

    ShmdataAnyReader::ptr reader = make_shared<ShmdataAnyReader>();
    reader->set_path (shmdata_socket_path);

    // This is the callback for when new clouds are received
    reader->set_callback([=](void* data, int size, unsigned long long timestamp, const char* type, void* /*unused*/)
    {
      vector<char> buffer((char*)data, (char*)data + size);
      if (string(type) == string(POINTCLOUD_TYPE_BASE))
        display_->setInputCloud(buffer, false, timestamp);
      else if (string(type) == string(POINTCLOUD_TYPE_COMPRESSED))
        display_->setInputCloud(buffer, true, timestamp);
    }, nullptr);

    reader->start ();
    register_shmdata (reader);
    
    return true;
  }

  bool
  PostureDisplay::disconnect_all ()
  {
    display_.reset();
    return true;
  }

  bool
  PostureDisplay::can_sink_caps (std::string caps)
  {
    return (caps == POINTCLOUD_TYPE_BASE)
      || (caps == POINTCLOUD_TYPE_COMPRESSED);
  }
  
} // end of namespace
