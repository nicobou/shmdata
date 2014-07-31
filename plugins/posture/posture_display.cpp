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

#define POINTCLOUD_TYPE_BASE          "application/x-pcl"
#define POINTCLOUD_TYPE_COMPRESSED    "application/x-pcd"

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
    //registering connect
    install_method ("Connect",
		    "connect",
		    "connect the sink to a shmdata socket", 
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "socket",
						  "shmdata socket path to connect with",
						  NULL),
		    (Method::method_ptr)&connect_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);
  
    //registering disconnect
    install_method ("Disconnect",
		    "disconnect",
		    "disconnect the sink from the shmdata socket", 
		    "success or fail",
		    Method::make_arg_description ("none",
						  NULL),
		    (Method::method_ptr)&disconnect, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    this);
  }

  PostureDisplay::~PostureDisplay()
  {
  }

  bool
  PostureDisplay::init()
  {
    init_segment (this);

    return true;
  }

  gboolean
  PostureDisplay::connect_wrapped (gpointer connector_name, gpointer user_data)
  {
    PostureDisplay* context = static_cast<PostureDisplay*>(user_data);
       
    if (context->connect ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
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

  gboolean
  PostureDisplay::disconnect (gpointer /*unused*/, gpointer user_data)
  {
    PostureDisplay* context = static_cast<PostureDisplay*>(user_data);
    context->display_.reset();
    context->clear_shmdatas ();
    return TRUE;
  }

} // end of namespace
