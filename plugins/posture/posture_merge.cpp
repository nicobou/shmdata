
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
    merger_ = make_shared<PointCloudMerger>();
  }

  PostureMerge::~PostureMerge()
  {
    stop();
  }

  bool
  PostureMerge::start()
  {
    merger_->setCalibrationPath(calibration_path_);
    merger_->setDevicesPath(devices_path_);
    merger_->setCompression(compress_cloud_);

    merger_->start();

    return true;
  }

  bool
  PostureMerge::stop()
  {
    merger_->stop();

    return true;
  }

  bool
  PostureMerge::init_segment()
  {
    init_startable (this);

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
