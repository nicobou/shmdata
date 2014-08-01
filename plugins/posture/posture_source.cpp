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

#include "posture_source.h"

#include <iostream>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSrc,
				       "3D camera",
				       "video source", 
				       "Grabs 3D data (point clouds / meshes) using a zcamera",
				       "LGPL",
				       "posturesrc",
				       "Emmanuel Durand");

  PostureSrc::PostureSrc() :
    custom_props_(std::make_shared<CustomPropertyHelper> ())
  {
    zcamera_ = make_shared<ZCamera>();

    zcamera_->setCallbackCloud(cb_frame_cloud, this);
    zcamera_->setCallbackDepth(cb_frame_depth, this);
    zcamera_->setCallbackRgb(cb_frame_rgb, this);
    zcamera_->setCallbackIR(cb_frame_ir, this);
  }

  PostureSrc::~PostureSrc()
  {
    stop();
  }

  bool
  PostureSrc::start()
  {
    zcamera_->setCalibrationPath(calibration_path_);
    zcamera_->setDevicesPath(devices_path_);
    zcamera_->setDeviceIndex(device_index_);
    zcamera_->setCaptureIR(capture_ir_);
    zcamera_->setCompression(compress_cloud_);
    zcamera_->setCaptureMode((posture::ZCamera::CaptureMode)capture_mode_);

    zcamera_->start();
    return true;
  }

  bool
  PostureSrc::stop()
  {
    zcamera_->stop();

    if (cloud_writer_.get() != nullptr)
      {
	unregister_shmdata(cloud_writer_->get_path());
	cloud_writer_.reset();
      }
    if (depth_writer_.get() != nullptr)
      {
	unregister_shmdata(depth_writer_->get_path());
	depth_writer_.reset();
      }
    if (rgb_writer_.get() != nullptr)
      {
	unregister_shmdata(rgb_writer_->get_path());
	rgb_writer_.reset();
      }
    if (ir_writer_.get() != nullptr)
      {
	unregister_shmdata(ir_writer_->get_path());
	ir_writer_.reset();
      }

    return true;
  }

  bool
  PostureSrc::init()
  {
    init_startable (this);
    init_segment (this);
    calibration_path_prop_ = custom_props_->make_string_property ("calibration_path",
								  "Path to the calibration file",
								  calibration_path_.c_str(),
								  (GParamFlags)G_PARAM_READWRITE,
								  PostureSrc::set_calibration_path,
								  PostureSrc::get_calibration_path,
								  this);
    install_property_by_pspec (custom_props_->get_gobject (),
			       calibration_path_prop_,
			       "calibration_path",
			       "Path to the calibration file");

    devices_path_prop_ = custom_props_->make_string_property ("devices_path",
							      "Path to the devices description file",
							      devices_path_.c_str(),
							      (GParamFlags)G_PARAM_READWRITE,
							      PostureSrc::set_devices_path,
							      PostureSrc::get_devices_path,
							      this);
    install_property_by_pspec (custom_props_->get_gobject (),
			       devices_path_prop_,
			       "devices",
			       "Path to the devices description file");

    device_index_prop_ = custom_props_->make_int_property ("device_index",
							   "Index of the device to use",
							   0,
							   7,
							   device_index_,
							   (GParamFlags)G_PARAM_READWRITE,
							   PostureSrc::set_device_index,
							   PostureSrc::get_device_index,
							   this);
    install_property_by_pspec (custom_props_->get_gobject (),
			       device_index_prop_,
			       "device_index",
			       "Index of the device to use");

    capture_ir_prop_ = custom_props_->make_boolean_property ("capture_ir",
							     "Grab the IR image if true",
							     capture_ir_,
							     (GParamFlags)G_PARAM_READWRITE,
							     PostureSrc::set_capture_ir,
							     PostureSrc::get_capture_ir,
							     this);
    install_property_by_pspec (custom_props_->get_gobject (),
			       capture_ir_prop_,
			       "capture_ir",
			       "Grab the IR image if true");

    compress_cloud_prop_ = custom_props_->make_boolean_property ("compress_cloud",
								 "Compress the cloud if true",
								 compress_cloud_,
								 (GParamFlags)G_PARAM_READWRITE,
								 PostureSrc::set_compress_cloud,
								 PostureSrc::get_compress_cloud,
								 this);
    install_property_by_pspec (custom_props_->get_gobject (),
			       compress_cloud_prop_,
			       "compress_cloud",
			       "Compress the cloud if true");

    capture_modes_enum_[0].value = 0;
    capture_modes_enum_[0].value_name = "Default mode";
    capture_modes_enum_[0].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[1].value = 1;
    capture_modes_enum_[1].value_name = "SXGA 15Hz";
    capture_modes_enum_[1].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[2].value = 2;
    capture_modes_enum_[2].value_name = "VGA 30Hz";
    capture_modes_enum_[2].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[3].value = 3;
    capture_modes_enum_[3].value_name = "VGA 25Hz";
    capture_modes_enum_[3].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[4].value = 4;
    capture_modes_enum_[4].value_name = "QVGA 25Hz";
    capture_modes_enum_[4].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[5].value = 5;
    capture_modes_enum_[5].value_name = "QVGA 30Hz";
    capture_modes_enum_[5].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[6].value = 6;
    capture_modes_enum_[6].value_name = "QVGA 60Hz";
    capture_modes_enum_[6].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[7].value = 7;
    capture_modes_enum_[7].value_name = "QQVGA 25Hz";
    capture_modes_enum_[7].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[8].value = 8;
    capture_modes_enum_[8].value_name = "QQVGA 30Hz";
    capture_modes_enum_[8].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[9].value = 9;
    capture_modes_enum_[9].value_name = "QQVGA 60Hz";
    capture_modes_enum_[9].value_nick = capture_modes_enum_[0].value_name;
    capture_modes_enum_[10].value = 0;
    capture_modes_enum_[10].value_name = nullptr;
    capture_modes_enum_[10].value_nick = nullptr;

    capture_mode_prop_ = custom_props_->make_enum_property ("capture_mode",
							    "Capture mode of the device",
							    0,
							    capture_modes_enum_,
							    (GParamFlags)G_PARAM_READWRITE,
							    PostureSrc::set_capture_mode,
							    PostureSrc::get_capture_mode,
							    this);
    install_property_by_pspec (custom_props_->get_gobject (),
			       capture_mode_prop_,
			       "capture_mode",
			       "Capture mode of the device");

    return true;
  }

  const gchar*
  PostureSrc::get_calibration_path(void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    return ctx->calibration_path_.c_str();
  }

  void
  PostureSrc::set_calibration_path(const gchar* name, void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    if (name != nullptr)
      ctx->calibration_path_ = name;
  }

  const gchar*
  PostureSrc::get_devices_path(void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    return ctx->devices_path_.c_str();
  }

  void
  PostureSrc::set_devices_path(const gchar* name, void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    if (name != nullptr)
      ctx->devices_path_ = name;
  }

  int
  PostureSrc::get_device_index(void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    return ctx->device_index_;
  }

  void
  PostureSrc::set_device_index(const int index, void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    ctx->device_index_ = index;
  }

  int
  PostureSrc::get_capture_ir(void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    return ctx->capture_ir_;
  }

  void
  PostureSrc::set_capture_ir(const int ir, void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    ctx->capture_ir_ = ir;
  }

  int
  PostureSrc::get_compress_cloud(void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    return ctx->compress_cloud_;
  }

  void
  PostureSrc::set_compress_cloud(const int compress, void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    ctx->compress_cloud_ = compress;
  }

  int
  PostureSrc::get_capture_mode(void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    return ctx->capture_mode_;
  }

  void
  PostureSrc::set_capture_mode(const int mode, void* user_data)
  {
    PostureSrc* ctx = (PostureSrc*)user_data;
    ctx->capture_mode_ = mode;
  }

  void
  PostureSrc::cb_frame_cloud(void* context, const vector<char>& data)
  {
    PostureSrc* ctx = (PostureSrc*)context;

    if (ctx->cloud_writer_.get() == nullptr)
      {
        ctx->cloud_writer_.reset(new ShmdataAnyWriter);
        ctx->cloud_writer_->set_path(ctx->make_file_name("cloud"));
        ctx->register_shmdata(ctx->cloud_writer_);
        if (ctx->compress_cloud_)
          ctx->cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_COMPRESSED));
        else
          ctx->cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_BASE));
        ctx->cloud_writer_->start();
      }

    ctx->cloud_buffers_[ctx->cloud_buffer_index_] = make_shared<vector<char>>(data);
    ctx->cloud_writer_->push_data_auto_clock((void*)ctx->cloud_buffers_[ctx->cloud_buffer_index_]->data(), data.size(), nullptr, nullptr);
    ctx->cloud_buffer_index_ = (ctx->cloud_buffer_index_ + 1) % 3;
  }

  void
  PostureSrc::cb_frame_depth(void* context, const vector<unsigned char>& data, int width, int height)
  {
    PostureSrc* ctx = (PostureSrc*)context;

    if (ctx->depth_writer_.get() == nullptr || ctx->depth_width_ != width || ctx->depth_height_ != height)
      {
        ctx->depth_writer_.reset(new ShmdataAnyWriter);
        ctx->depth_writer_->set_path(ctx->make_file_name("depth"));
        ctx->register_shmdata(ctx->depth_writer_);
        ctx->depth_width_ = width;
        ctx->depth_height_ = height;
        
        char buffer[256] = "";
        sprintf(buffer, "video/x-raw-gray,bpp=16,endianness=1234,depth=16,width=%i,height=%i,framerate=30/1", width, height);
        ctx->depth_writer_->set_data_type(string(buffer));
        ctx->depth_writer_->start();
      }

    ctx->depth_writer_->push_data_auto_clock((void*)data.data(), width * height * 2, nullptr, nullptr);
  }

  void
  PostureSrc::cb_frame_rgb(void* context, const vector<unsigned char>& data, int width, int height)
  {
    PostureSrc* ctx = (PostureSrc*)context;

    if (ctx->rgb_writer_.get() == nullptr || ctx->rgb_width_ != width || ctx->rgb_height_ != height)
      {
        ctx->rgb_writer_.reset(new ShmdataAnyWriter);
        ctx->rgb_writer_->set_path(ctx->make_file_name("rgb"));
        ctx->register_shmdata(ctx->rgb_writer_);
        ctx->rgb_width_ = width;
        ctx->rgb_height_ = height;
        
        char buffer[256] = "";
        sprintf(buffer, "video/x-raw-rgb,bpp=24,endianness=4321,depth=24,red_mask=16711680,green_mask=65280,blue_mask=255,width=%i,height=%i,framerate=30/1", width, height);
        ctx->rgb_writer_->set_data_type(string(buffer));
        ctx->rgb_writer_->start();
      }

    ctx->rgb_writer_->push_data_auto_clock((void*)data.data(), width * height * 3, nullptr, nullptr);
  }

  void
  PostureSrc::cb_frame_ir(void* context, const vector<unsigned char>& data, int width, int height)
  {
    PostureSrc* ctx = (PostureSrc*)context;

    if (ctx->ir_writer_.get() == nullptr || ctx->ir_width_ != width || ctx->ir_height_ != height)
      {
        ctx->ir_writer_.reset(new ShmdataAnyWriter);
        ctx->ir_writer_->set_path(ctx->make_file_name("ir"));
        ctx->register_shmdata(ctx->ir_writer_);
        ctx->ir_width_ = width;
        ctx->ir_height_ = height;
        
        char buffer[256] = "";
        sprintf(buffer, "video/x-raw-gray,bpp=16,endianness=1234,depth=16,width=%i,height=%i,framerate=30/1", width, height);
        ctx->ir_writer_->set_data_type(string(buffer));
        ctx->ir_writer_->start();
      }

    ctx->ir_writer_->push_data_auto_clock((void*)data.data(), width * height * 2, nullptr, nullptr);
  }
}
