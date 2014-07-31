
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

#include "posture_merge.h"

#include <iostream>

#define POINTCLOUD_TYPE_BASE          "application/x-pcl"
#define POINTCLOUD_TYPE_COMPRESSED    "application/x-pcd"

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureMerge,
				       "Point clouds merge",
				       "video sink", 
				       "Merges point clouds captured with 3D cameras",
				       "LGPL",
				       "posturemerge",				
				       "Emmanuel Durand");

  PostureMerge::PostureMerge() :
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
		    "disconnect-all",
		    "disconnect the sink from the shmdata socket", 
		    "success or fail",
		    Method::make_arg_description ("none",
						  NULL),
		    (Method::method_ptr)&disconnect, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    this);
  }

  PostureMerge::~PostureMerge()
  {
    stop();
  }

  bool
  PostureMerge::start()
  {
    merger_ = make_shared<PointCloudMerger>("", source_id_);

    merger_->setCalibrationPath(calibration_path_);
    merger_->setDevicesPath(devices_path_);
    merger_->setCompression(compress_cloud_);

    merger_->start();

    return true;
  }

  bool
  PostureMerge::stop()
  {
    if (merger_.get() != nullptr)
      merger_->stop();

    if (cloud_writer_.get() != nullptr)
    {
      unregister_shmdata(cloud_writer_->get_path());
      cloud_writer_.reset();
    }

    return true;
  }

  bool
  PostureMerge::init()
  {
    init_startable (this);
    init_segment (this);

    calibration_path_prop_ = custom_props_->make_string_property ("calibration_path",
      "Path to the calibration file",
      calibration_path_.c_str(),
      (GParamFlags)G_PARAM_READWRITE,
      PostureMerge::set_calibration_path,
      PostureMerge::get_calibration_path,
      this);
    install_property_by_pspec (custom_props_->get_gobject (),
      calibration_path_prop_,
      "calibration_path",
      "Path to the calibration file");

    devices_path_prop_ = custom_props_->make_string_property ("devices_path",
      "Path to the devices description file",
      devices_path_.c_str(),
      (GParamFlags)G_PARAM_READWRITE,
      PostureMerge::set_devices_path,
      PostureMerge::get_devices_path,
      this);
    install_property_by_pspec (custom_props_->get_gobject (),
      devices_path_prop_,
      "devices",
      "Path to the devices description file");

    compress_cloud_prop_ = custom_props_->make_boolean_property ("compress_cloud",
      "Compress the cloud if true",
      compress_cloud_,
      (GParamFlags)G_PARAM_READWRITE,
      PostureMerge::set_compress_cloud,
      PostureMerge::get_compress_cloud,
      this);
    install_property_by_pspec (custom_props_->get_gobject (),
      compress_cloud_prop_,
      "compress_cloud",
      "Compress the cloud if true");

    return true;
  }

  gboolean
  PostureMerge::connect_wrapped (gpointer connector_name, gpointer user_data)
  {
    PostureMerge* context = static_cast<PostureMerge*>(user_data);
       
    if (context->connect ((char *)connector_name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  PostureMerge::connect (std::string shmdata_socket_path)
  {
    int index = source_id_;
    source_id_ += 1;
    ShmdataAnyReader::ptr reader_ = make_shared<ShmdataAnyReader>();
    reader_->set_path (shmdata_socket_path);

    // This is the callback for when new clouds are received
    reader_->set_callback([=](void* data, int size, unsigned long long timestamp, const char* type, void* user_data)
    {
      if (!mutex_.try_lock())
        return;

      if (string(type) == string(POINTCLOUD_TYPE_BASE))
        merger_->setInputCloud(index, vector<char>((char*)data, (char*)data + size), false, timestamp);
      else if (string(type) == string(POINTCLOUD_TYPE_COMPRESSED))
        merger_->setInputCloud(index, vector<char>((char*)data, (char*)data + size), true, timestamp);
      else
        return;

      merged_cloud_ = merger_->getCloud();

      if (cloud_writer_.get() == nullptr)
      {
        cloud_writer_.reset(new ShmdataAnyWriter);
        cloud_writer_->set_path(make_file_name("cloud"));
        register_shmdata(cloud_writer_);
        if (compress_cloud_)
          cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_COMPRESSED));
        else
          cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_BASE));
        cloud_writer_->start();
      }

      cloud_writer_->push_data_auto_clock((void*)merged_cloud_.data(), merged_cloud_.size(), NULL, NULL);

      mutex_.unlock();
    }, nullptr);

    reader_->start ();
    register_shmdata (reader_);
    
    return true;
  }

  gboolean
  PostureMerge::disconnect (gpointer /*unused*/, gpointer user_data)
  {
    PostureMerge* context = static_cast<PostureMerge*>(user_data);
    context->clear_shmdatas ();
    context->source_id_ = 0;
    return TRUE;
  }

  const gchar*
  PostureMerge::get_calibration_path(void* user_data)
  {
    PostureMerge* ctx = (PostureMerge*)user_data;
    return ctx->calibration_path_.c_str();
  }

  void
  PostureMerge::set_calibration_path(const gchar* name, void* user_data)
  {
    PostureMerge* ctx = (PostureMerge*)user_data;
    if (name != nullptr)
      ctx->calibration_path_ = name;
  }

  const gchar*
  PostureMerge::get_devices_path(void* user_data)
  {
    PostureMerge* ctx = (PostureMerge*)user_data;
    return ctx->devices_path_.c_str();
  }

  void
  PostureMerge::set_devices_path(const gchar* name, void* user_data)
  {
    PostureMerge* ctx = (PostureMerge*)user_data;
    if (name != nullptr)
      ctx->devices_path_ = name;
  }

  int
  PostureMerge::get_compress_cloud(void* user_data)
  {
    PostureMerge* ctx = (PostureMerge*)user_data;
    return ctx->compress_cloud_;
  }

  void
  PostureMerge::set_compress_cloud(const int compress, void* user_data)
  {
    PostureMerge* ctx = (PostureMerge*)user_data;
    ctx->compress_cloud_ = compress;
  }

} // end of namespace
