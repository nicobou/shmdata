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

#include "./posture_source.hpp"
#include "switcher/std2.hpp"

#include <functional>
#include <iostream>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PostureSrc,
    "posturesrc",
    "Depth Camera",
    "video",
    "writer/device",
    "Grabs point clouds/meshes using a zcamera",
    "LGPL",
    "Emmanuel Durand");

PostureSrc::PostureSrc(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
  zcamera_ = std2::make_unique<ZCamera> ();
  
  zcamera_->setCallbackCloud(cb_frame_cloud, this);
  zcamera_->setCallbackMesh(cb_frame_mesh, this);
  zcamera_->setCallbackDepth(cb_frame_depth, this);
  zcamera_->setCallbackRgb(cb_frame_rgb, this);
  zcamera_->setCallbackIR(cb_frame_ir, this);
}

PostureSrc::~PostureSrc() {
  stop();
}

bool
PostureSrc::start() {
  zcamera_->setCalibrationPath(calibration_path_);
  zcamera_->setDevicesPath(devices_path_);
  zcamera_->setDeviceIndex(device_index_);
  zcamera_->setCaptureIR(capture_ir_);
  zcamera_->setBuildMesh(build_mesh_);
  zcamera_->setCompression(compress_cloud_);
  zcamera_->setCaptureMode((posture::ZCamera::CaptureMode) capture_mode_);
  zcamera_->setOutlierFilterParameters(filter_outliers_, filter_mean_k_, filter_stddev_mul_);

  zcamera_->start();

  rgb_focal_ = zcamera_->getRGBFocal();
  depth_focal_ = zcamera_->getDepthFocal();
  // TODO: G_PARAM_READABLE are not displayed in Scenic2
  rgb_focal_prop_ = custom_props_->make_double_property("rgb_focal",
                              "RGB focal length",
                              0.0,
                              10000.0,
                              rgb_focal_,
                              (GParamFlags)
                              G_PARAM_READABLE,
                              PostureSrc::nope,
                              PostureSrc::get_rgb_focal,
                              this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            rgb_focal_prop_, "rgb_focal",
                            "RGB focal length");

  depth_focal_prop_ = custom_props_->make_double_property("depth_focal",
                              "Depth focal length",
                              0.0,
                              10000.0,
                              depth_focal_,
                              (GParamFlags)
                              G_PARAM_READWRITE,
                              PostureSrc::set_depth_focal,
                              PostureSrc::get_depth_focal,
                              this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            depth_focal_prop_, "depth_focal",
                            "Depth focal length");

  return true;
}

bool
PostureSrc::stop() {
  zcamera_->stop();

  cloud_writer_.reset();
  mesh_writer_.reset();
  depth_writer_.reset();
  rgb_writer_.reset();
  ir_writer_.reset();

  uninstall_property("rgb_focal_x");
  uninstall_property("rgb_focal_y");

  return true;
}

bool
PostureSrc::init() {
  init_startable(this);

  calibration_path_prop_ =
      custom_props_->make_string_property("calibration_path",
                            "Path to the calibration file",
                            calibration_path_.c_str(),
                            (GParamFlags) G_PARAM_READWRITE,
                            PostureSrc::set_calibration_path,
                            PostureSrc::get_calibration_path,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            calibration_path_prop_, "calibration_path",
                            "Path to the calibration file");

  devices_path_prop_ = custom_props_->make_string_property("devices_path",
                            "Path to the devices description file",
                            devices_path_.c_str
                            (), (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSrc::set_devices_path,
                            PostureSrc::get_devices_path,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            devices_path_prop_, "devices",
                            "Path to the devices description file");

  device_index_prop_ = custom_props_->make_int_property("device_index",
                            "Index of the device to use",
                            0,
                            7,
                            device_index_,
                            (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSrc::set_device_index,
                            PostureSrc::get_device_index,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            device_index_prop_, "device_index",
                            "Index of the device to use");

  capture_ir_prop_ = custom_props_->make_boolean_property("capture_ir",
                            "Grab the IR image if true",
                            capture_ir_,
                            (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSrc::set_capture_ir,
                            PostureSrc::get_capture_ir,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            capture_ir_prop_, "capture_ir",
                            "Grab the IR image if true");

  build_mesh_prop_ = custom_props_->make_boolean_property("build_mesh",
                            "Build a mesh from the cloud",
                            build_mesh_,
                            (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSrc::set_build_mesh,
                            PostureSrc::get_build_mesh,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            build_mesh_prop_, "build_mesh",
                            "Build a mesh from the cloud");

  compress_cloud_prop_ = custom_props_->make_boolean_property("compress_cloud",
                            "Compress the cloud if true",
                            compress_cloud_,
                            (GParamFlags) G_PARAM_READWRITE,
                            PostureSrc::set_compress_cloud,
                            PostureSrc::get_compress_cloud,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            compress_cloud_prop_, "compress_cloud",
                            "Compress the cloud if true");

  reload_calibration_prop_ = custom_props_->make_boolean_property("reload_calibration",
                                "Reload calibration at each frame",
                                reload_calibration_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureSrc::set_reload_calibration,
                                PostureSrc::get_reload_calibration,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            reload_calibration_prop_, "reload_calibration",
                            "Reload calibration at each frame");

  downsample_prop_ = custom_props_->make_boolean_property("downsample",
                                "Activate the cloud downsampling",
                                downsample_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureSrc::set_downsample_active,
                                PostureSrc::get_downsample_active,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            downsample_prop_, "downsample",
                            "Activate the cloud downsampling");

  filter_outliers_prop_ = custom_props_->make_boolean_property("filter_outliers",
                                "Filter outlier points from the cloud",
                                filter_outliers_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureSrc::set_filter_outliers,
                                PostureSrc::get_filter_outliers,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            filter_outliers_prop_, "filter_outliers",
                            "Filter outlier points from the cloud");

  capture_modes_enum_[0].value = 0;
  capture_modes_enum_[0].value_name = "Default mode";
  capture_modes_enum_[0].value_nick = capture_modes_enum_[0].value_name;
  capture_modes_enum_[1].value = 1;
  capture_modes_enum_[1].value_name = "SXGA 15Hz";
  capture_modes_enum_[1].value_nick = capture_modes_enum_[1].value_name;
  capture_modes_enum_[2].value = 2;
  capture_modes_enum_[2].value_name = "VGA 30Hz";
  capture_modes_enum_[2].value_nick = capture_modes_enum_[2].value_name;
  capture_modes_enum_[3].value = 3;
  capture_modes_enum_[3].value_name = "VGA 25Hz";
  capture_modes_enum_[3].value_nick = capture_modes_enum_[3].value_name;
  capture_modes_enum_[4].value = 4;
  capture_modes_enum_[4].value_name = "QVGA 25Hz";
  capture_modes_enum_[4].value_nick = capture_modes_enum_[4].value_name;
  capture_modes_enum_[5].value = 5;
  capture_modes_enum_[5].value_name = "QVGA 30Hz";
  capture_modes_enum_[5].value_nick = capture_modes_enum_[5].value_name;
  capture_modes_enum_[6].value = 6;
  capture_modes_enum_[6].value_name = "QVGA 60Hz";
  capture_modes_enum_[6].value_nick = capture_modes_enum_[6].value_name;
  capture_modes_enum_[7].value = 7;
  capture_modes_enum_[7].value_name = "QQVGA 25Hz";
  capture_modes_enum_[7].value_nick = capture_modes_enum_[7].value_name;
  capture_modes_enum_[8].value = 8;
  capture_modes_enum_[8].value_name = "QQVGA 30Hz";
  capture_modes_enum_[8].value_nick = capture_modes_enum_[8].value_name;
  capture_modes_enum_[9].value = 9;
  capture_modes_enum_[9].value_name = "QQVGA 60Hz";
  capture_modes_enum_[9].value_nick = capture_modes_enum_[9].value_name;
  capture_modes_enum_[10].value = 0;
  capture_modes_enum_[10].value_name = nullptr;
  capture_modes_enum_[10].value_nick = nullptr;

  capture_mode_prop_ = custom_props_->make_enum_property("capture_mode",
                                                         "Capture mode of the device",
                                                         0,
                                                         capture_modes_enum_,
                                                         (GParamFlags)
                                                         G_PARAM_READWRITE,
                                                         PostureSrc::set_capture_mode,
                                                         PostureSrc::get_capture_mode,
                                                         this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            capture_mode_prop_, "capture_mode",
                            "Capture mode of the device");

  return true;
}

const gchar *
PostureSrc::get_calibration_path(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->calibration_path_.c_str();
}

void
PostureSrc::set_calibration_path(const gchar *name, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  if (name != nullptr)
    ctx->calibration_path_ = name;
}

const gchar *
PostureSrc::get_devices_path(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->devices_path_.c_str();
}

void
PostureSrc::set_devices_path(const gchar *name, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  if (name != nullptr)
    ctx->devices_path_ = name;
}

int
PostureSrc::get_device_index(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->device_index_;
}

void
PostureSrc::set_device_index(const int index, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->device_index_ = index;
}

int
PostureSrc::get_capture_ir(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->capture_ir_;
}

void
PostureSrc::set_capture_ir(const int ir, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->capture_ir_ = ir;
}

int
PostureSrc::get_build_mesh(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->build_mesh_;
}

void
PostureSrc::set_build_mesh(const int build_mesh, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;

  if (ctx->build_mesh_ != build_mesh && build_mesh == true)
  {
    ctx->build_mesh_ = build_mesh;

    ctx->build_mesh_edge_length_prop_ = ctx->custom_props_->make_int_property("build_mesh_edge_length",
                                "Edge length of the build mesh, in pixels",
                                1,
                                16,
                                ctx->build_mesh_edge_length_,
                                (GParamFlags)
                                G_PARAM_READWRITE,
                                PostureSrc::set_build_mesh_edge_length,
                                PostureSrc::get_build_mesh_edge_length,
                                ctx);
    ctx->install_property_by_pspec(ctx->custom_props_->get_gobject(),
                              ctx->build_mesh_edge_length_prop_, "build_mesh_edge_length",
                              "Edge length of the  build mesh, in pixels");
  }
  else if (ctx->build_mesh_edge_length_ != build_mesh && build_mesh == false)
  {
    ctx->build_mesh_edge_length_ = false;
    ctx->uninstall_property("build_mesh_edge_length");
  }

  if (ctx->zcamera_)
    ctx->zcamera_->setBuildEdgeLength(ctx->build_mesh_edge_length_);
}

int
PostureSrc::get_build_mesh_edge_length(void *user_data) {
    PostureSrc *ctx = (PostureSrc*) user_data;
    return ctx->build_mesh_edge_length_;
}

void
PostureSrc::set_build_mesh_edge_length(const int edge_length, void *user_data) {
    PostureSrc *ctx = (PostureSrc*) user_data;
    ctx->build_mesh_edge_length_ = edge_length;

    if (ctx->zcamera_)
        ctx->zcamera_->setBuildEdgeLength(ctx->build_mesh_edge_length_);
}

int
PostureSrc::get_compress_cloud(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->compress_cloud_;
}

void
PostureSrc::set_compress_cloud(const int compress, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->compress_cloud_ = compress;
}

int
PostureSrc::get_capture_mode(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->capture_mode_;
}

void
PostureSrc::set_capture_mode(const int mode, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->capture_mode_ = mode;
}

int
PostureSrc::get_reload_calibration(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->reload_calibration_;
}

void
PostureSrc::set_reload_calibration(const int reload, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->reload_calibration_ = reload;
}

int
PostureSrc::get_downsample_active(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->downsample_;
}

void
PostureSrc::set_downsample_active(const int active, void *user_data){
  PostureSrc *ctx = (PostureSrc *) user_data;

  if (ctx->downsample_ != active && active == true)
  {
    ctx->downsample_ = active;

    ctx->downsample_resolution_prop_ = ctx->custom_props_->make_double_property("downsample_resolution",
                                "Resampling resolution",
                                0.01,
                                1.0,
                                ctx->downsample_resolution_,
                                (GParamFlags)
                                G_PARAM_READWRITE,
                                PostureSrc::set_downsampling_resolution,
                                PostureSrc::get_downsampling_resolution,
                                ctx);
    ctx->install_property_by_pspec(ctx->custom_props_->get_gobject(),
                              ctx->downsample_resolution_prop_, "downsample_resolution",
                              "Resampling resolution");
  }
  else if (ctx->downsample_ != active && active == false)
  {
    ctx->downsample_ = false;
    ctx->uninstall_property("downsample_resolution");
  }

  if (ctx->zcamera_)
    ctx->zcamera_->setDownsampling(ctx->downsample_, ctx->downsample_resolution_);
}

double
PostureSrc::get_downsampling_resolution(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->downsample_resolution_;
}

void
PostureSrc::set_downsampling_resolution(const double resolution, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->downsample_resolution_ = resolution;

  if (ctx->zcamera_)
    ctx->zcamera_->setDownsampling(ctx->downsample_, ctx->downsample_resolution_);
}

int
PostureSrc::get_filter_outliers(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->filter_outliers_;
}

void
PostureSrc::set_filter_outliers(const int active, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;

  if (ctx->filter_outliers_ != active && active == true)
  {
    ctx->filter_outliers_ = active;
    ctx->filter_mean_k_prop_ = ctx->custom_props_->make_int_property("filter_mean_k",
                              "Number of points to consider for the outlier filtering",
                              0,
                              16,
                              ctx->filter_mean_k_,
                              (GParamFlags)
                              G_PARAM_READWRITE,
                              PostureSrc::set_filter_mean_k,
                              PostureSrc::get_filter_mean_k,
                              ctx);
    ctx->install_property_by_pspec(ctx->custom_props_->get_gobject(),
                              ctx->filter_mean_k_prop_, "filter_mean_k",
                              "Number of points to consider for the outlier filtering");

    ctx->filter_stddev_mul_prop_ = ctx->custom_props_->make_double_property("filter_stddev_mul",
                                "Multiplier threshold on the statistical outlier filter (see PCL doc)",
                                0.0,
                                8.0,
                                ctx->filter_stddev_mul_,
                                (GParamFlags)
                                G_PARAM_READWRITE,
                                PostureSrc::set_filter_stddev_mul,
                                PostureSrc::get_filter_stddev_mul,
                                ctx);
    ctx->install_property_by_pspec(ctx->custom_props_->get_gobject(),
                              ctx->filter_stddev_mul_prop_, "filter_stddev_mul",
                              "Multiplier threshold on the statistical outlier filter (see PCL doc)");
  } 
  else if (ctx->filter_outliers_ != active && active == false)
  {
    ctx->filter_outliers_ = active;
    ctx->uninstall_property("filter_mean_k");
    ctx->uninstall_property("filter_stddev_mul");
  }

  if (ctx->zcamera_)
    ctx->zcamera_->setOutlierFilterParameters(ctx->filter_outliers_, ctx->filter_mean_k_, ctx->filter_stddev_mul_);
}

int
PostureSrc::get_filter_mean_k(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->filter_mean_k_;
}

void
PostureSrc::set_filter_mean_k(const int mean_k, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->filter_mean_k_ = mean_k;

  if (ctx->zcamera_)
    ctx->zcamera_->setOutlierFilterParameters(ctx->filter_outliers_, ctx->filter_mean_k_, ctx->filter_stddev_mul_);
}

double
PostureSrc::get_filter_stddev_mul(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->filter_stddev_mul_;
}

void
PostureSrc::set_filter_stddev_mul(const double stddev_mul, void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->filter_stddev_mul_ = stddev_mul;

  if (ctx->zcamera_)
    ctx->zcamera_->setOutlierFilterParameters(ctx->filter_outliers_, ctx->filter_mean_k_, ctx->filter_stddev_mul_);
}

double
PostureSrc::get_rgb_focal(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  return ctx->rgb_focal_;
}

void
PostureSrc::set_depth_focal(const double focal, void *user_data) {
    PostureSrc *ctx = (PostureSrc *) user_data;
    ctx->zcamera_->setDepthFocal(focal);
    ctx->depth_focal_ = ctx->zcamera_->getDepthFocal();
}

double
PostureSrc::get_depth_focal(void *user_data) {
  PostureSrc *ctx = (PostureSrc *) user_data;
  ctx->depth_focal_ = ctx->zcamera_->getDepthFocal();
  return ctx->depth_focal_;
}

void
PostureSrc::nope(const double /*unused*/, void* /*unused*/)
{
  return;
}

void
PostureSrc::cb_frame_cloud(void *context, const vector<char>&& data) {
  PostureSrc *ctx = (PostureSrc *) context;

  if (!ctx->cloud_writer_ || data.size() > ctx->cloud_writer_->writer(&shmdata::Writer::alloc_size)) {
    auto data_type = ctx->compress_cloud_ ? string(POINTCLOUD_TYPE_COMPRESSED) : string(POINTCLOUD_TYPE_BASE);
    ctx->cloud_writer_.reset();
    ctx->cloud_writer_ = std2::make_unique<ShmdataWriter>(ctx,
                                                    ctx->make_file_name("cloud"),
                                                    data.size() * 2,
                                                    data_type);

    if (!ctx->cloud_writer_) {
      g_warning("Unable to create mesh callback");
      return;
    }
  }

  if (ctx->reload_calibration_)
    ctx->zcamera_->reloadCalibration();

  ctx->cloud_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<char*>(data.data()), data.size());
  ctx->cloud_writer_->bytes_written(data.size());
}

void
PostureSrc::cb_frame_mesh(void* context, vector<unsigned char>&& data)
{
  PostureSrc *ctx = static_cast<PostureSrc*>(context);
  
  if (!ctx->mesh_writer_ || data.size() > ctx->mesh_writer_->writer(&shmdata::Writer::alloc_size)) {
    ctx->mesh_writer_.reset();
    ctx->mesh_writer_ = std2::make_unique<ShmdataWriter>(ctx,
                                                     ctx->make_file_name("mesh"),
                                                     data.size() * 2,
                                                     string(POLYGONMESH_TYPE_BASE));

    if (!ctx->mesh_writer_) {
      g_warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->mesh_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<unsigned char*>(data.data()), data.size());
  ctx->mesh_writer_->bytes_written(data.size());
}

void
PostureSrc::cb_frame_depth(void *context,
                           const vector<unsigned char>&data, int width,
                           int height) {
  PostureSrc *ctx = (PostureSrc *) context;

  if (!ctx->depth_writer_ || ctx->depth_width_ != width || ctx->depth_height_ != height) {
    ctx->depth_width_ = width;
    ctx->depth_height_ = height;
    char buffer[256] = "";
    sprintf(buffer, "video/x-raw,format=(string)GRAY16_BE,width=(int)%i,height=(int)%i,framerate=30/1", width, height);

    ctx->depth_writer_.reset();
    ctx->depth_writer_ = std2::make_unique<ShmdataWriter>(ctx,
                                                    ctx->make_file_name("depth"),
                                                    data.size(),
                                                    string(buffer));

    if (!ctx->depth_writer_) {
      g_warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->depth_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<unsigned char*>(data.data()), data.size());
  ctx->depth_writer_->bytes_written(data.size());
}

void
PostureSrc::cb_frame_rgb(void *context,
                         const vector<unsigned char>&data, int width,
                         int height) {
  using namespace std::placeholders;

  PostureSrc *ctx = (PostureSrc *) context;

  if (!ctx->rgb_writer_ || ctx->rgb_width_ != width || ctx->rgb_height_ != height) {
    ctx->rgb_width_ = width;
    ctx->rgb_height_ = height;
    char buffer[256] = "";
    sprintf(buffer, "video/x-raw,format=(string)BGR,width=(int)%i,height=(int)%i,framerate=30/1", width, height);

    ctx->rgb_writer_.reset();
    ctx->rgb_writer_ = std2::make_unique<ShmdataWriter>(ctx,
                                                  ctx->make_file_name("rgb"),
                                                  data.size(),
                                                  string(buffer));

    if (!ctx->rgb_writer_) {
      g_warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->rgb_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<unsigned char*>(data.data()), data.size());
  ctx->rgb_writer_->bytes_written(data.size());
}

void
PostureSrc::cb_frame_ir(void *context, const vector<unsigned char>&data,
                        int width, int height) {
  PostureSrc *ctx = (PostureSrc *) context;

  if (!ctx->ir_writer_ || ctx->ir_width_ != width || ctx->ir_height_ != height) {
    ctx->ir_width_ = width;
    ctx->ir_height_ = height;

    char buffer[256] = "";
    sprintf(buffer, "video/x-raw,format=(string)GRAY16_BE,width=(int)%i,height=(int)%i,framerate=30/1", width, height);

    ctx->ir_writer_.reset();
    ctx->ir_writer_ = std2::make_unique<ShmdataWriter>(ctx,
                                                 ctx->make_file_name("ir"),
                                                 data.size(),
                                                 string(buffer));

    if (!ctx->ir_writer_) {
      g_warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->ir_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<unsigned char*>(data.data()), data.size());
  ctx->ir_writer_->bytes_written(data.size());
}

}
