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

#include <functional>
#include <iostream>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSrc,
                                     "posturesrc",
                                     "Depth Camera",
                                     "video",
                                     "writer/device",
                                     "Grabs point clouds/meshes using a zcamera",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureSrc::PostureSrc(QuiddityConfiguration&&) {
  calibration_reader_ = std::make_unique<CalibrationReader>("default.kvc");
  zcamera_ = std::make_unique<ZCamera>();

  zcamera_->setCallbackCloud(cb_frame_cloud, this);
  zcamera_->setCallbackMesh(cb_frame_mesh, this);
  zcamera_->setCallbackDepth(cb_frame_depth, this);
  zcamera_->setCallbackRgb(cb_frame_rgb, this);
  zcamera_->setCallbackIR(cb_frame_ir, this);
  init_startable(this);

  pmanage<MPtr(&PContainer::make_string)>("calibration_path",
                                          [this](const std::string& val) {
                                            calibration_path_ = val;
                                            return true;
                                          },
                                          [this]() { return calibration_path_; },
                                          "Calibration path",
                                          "Path to the calibration file",
                                          calibration_path_);

  pmanage<MPtr(&PContainer::make_int)>("device_index",
                                       [this](const int& val) {
                                         device_index_ = val;
                                         return true;
                                       },
                                       [this]() { return device_index_; },
                                       "Device index",
                                       "Index of the device to use",
                                       device_index_,
                                       0,
                                       ZCamera::getDeviceCount() - 1);

  pmanage<MPtr(&PContainer::make_bool)>("random_data",
                                        [this](const bool& val) {
                                          if (!is_started()) random_data_ = val;
                                          return true;
                                        },
                                        [this]() { return random_data_; },
                                        "Random data",
                                        "Generate random data",
                                        random_data_);

  pmanage<MPtr(&PContainer::make_bool)>("capture_ir",
                                        [this](const bool& val) {
                                          capture_ir_ = val;
                                          return true;
                                        },
                                        [this]() { return capture_ir_; },
                                        "Capture ir",
                                        "Grab the IR image if true",
                                        capture_ir_);

  pmanage<MPtr(&PContainer::make_bool)>(
      "build_mesh",
      [this](const bool& val) {
        if (build_mesh_ != val && val == true) {
          build_mesh_ = val;
          build_mesh_edge_length_id_ = pmanage<MPtr(&PContainer::make_int)>(
              "build_mesh_edge_length",
              [this](const int& val) {
                build_mesh_edge_length_ = val;
                if (zcamera_) zcamera_->setBuildEdgeLength(build_mesh_edge_length_);
                return true;
              },
              [this]() { return build_mesh_edge_length_; },
              "Build mesh edge length",
              "Edge length of the build mesh, in pixels",
              build_mesh_edge_length_,
              1,
              16);
        } else if (build_mesh_edge_length_ != val && val == false) {
          build_mesh_edge_length_ = false;
          pmanage<MPtr(&PContainer::remove)>(build_mesh_edge_length_id_);
        }
        if (zcamera_) zcamera_->setBuildEdgeLength(build_mesh_edge_length_);
        return true;
      },
      [this]() { return build_mesh_; },
      "Build mesh",
      "Build a mesh from the cloud",
      build_mesh_);

  pmanage<MPtr(&PContainer::make_bool)>("compress_cloud",
                                        [this](const bool& val) {
                                          compress_cloud_ = val;
                                          return true;
                                        },
                                        [this]() { return compress_cloud_; },
                                        "Compress cloud",
                                        "Compress the cloud if true",
                                        compress_cloud_);

  pmanage<MPtr(&PContainer::make_bool)>("reload_calibration",
                                        [this](const bool& val) {
                                          reload_calibration_ = val;
                                          return true;
                                        },
                                        [this]() { return reload_calibration_; },
                                        "Pre frame calibration",
                                        "Reload calibration at each frame",
                                        reload_calibration_);

  pmanage<MPtr(&PContainer::make_bool)>(
      "downsample",
      [this](const bool& val) {
        if (downsample_ != val && val == true) {
          downsample_ = val;
          downsample_resolution_id_ = pmanage<MPtr(&PContainer::make_double)>(
              "downsample_resolution",
              [this](const double& val) {
                downsample_resolution_ = val;
                if (zcamera_) zcamera_->setDownsampling(downsample_, downsample_resolution_);
                return true;
              },
              [this]() { return downsample_resolution_; },
              "Resampling resolution",
              "Resampling resolution",
              downsample_resolution_,
              0.01,
              1.0);
        } else if (downsample_ != val && val == false) {
          downsample_ = false;
          pmanage<MPtr(&PContainer::remove)>(downsample_resolution_id_);
        }
        if (zcamera_) zcamera_->setDownsampling(downsample_, downsample_resolution_);
        return true;
      },
      [this]() { return downsample_; },
      "Downsample",
      "Activate the cloud downsampling",
      downsample_);
  //
  // Filtering
  pmanage<MPtr(&PContainer::make_group)>("filtering", "Filtering", "Filtering");

  pmanage<MPtr(&PContainer::make_parented_int)>("bilateral_filter_kernel_size",
                                                "filtering",
                                                [this](const int& val) {
                                                  bilateral_filter_kernel_size_ = val;
                                                  if (zcamera_)
                                                    zcamera_->setBilateralFiltering(
                                                        bilateral_filter_kernel_size_,
                                                        bilateral_filter_sigma_pos_,
                                                        bilateral_filter_sigma_value_,
                                                        bilateral_filter_iterations_);
                                                  return true;
                                                },
                                                [this]() { return bilateral_filter_kernel_size_; },
                                                "Filter kernel size",
                                                "Depth map filter kernel size",
                                                bilateral_filter_kernel_size_,
                                                1,
                                                32);

  pmanage<MPtr(&PContainer::make_parented_float)>("bilateral_filter_sigma_pos_",
                                                  "filtering",
                                                  [this](const float& val) {
                                                    bilateral_filter_sigma_pos_ = val;
                                                    if (zcamera_)
                                                      zcamera_->setBilateralFiltering(
                                                          bilateral_filter_kernel_size_,
                                                          bilateral_filter_sigma_pos_,
                                                          bilateral_filter_sigma_value_,
                                                          bilateral_filter_iterations_);
                                                    return true;
                                                  },
                                                  [this]() { return bilateral_filter_sigma_pos_; },
                                                  "Filter spatial sigma",
                                                  "Depth map filter spatial sigma",
                                                  bilateral_filter_sigma_pos_,
                                                  0.1,
                                                  16.0);

  pmanage<MPtr(&PContainer::make_parented_float)>(
      "bilateral_filter_sigma_value_",
      "filtering",
      [this](const float& val) {
        bilateral_filter_sigma_value_ = val;
        if (zcamera_)
          zcamera_->setBilateralFiltering(bilateral_filter_kernel_size_,
                                          bilateral_filter_sigma_pos_,
                                          bilateral_filter_sigma_value_,
                                          bilateral_filter_iterations_);
        return true;
      },
      [this]() { return bilateral_filter_sigma_value_; },
      "Filter value sigma",
      "Depth map filter value sigma",
      bilateral_filter_sigma_value_,
      1.0,
      2000.0);

  pmanage<MPtr(&PContainer::make_parented_int)>("bilateral_filter_iterations",
                                                "filtering",
                                                [this](const int& val) {
                                                  bilateral_filter_iterations_ = val;
                                                  if (zcamera_)
                                                    zcamera_->setBilateralFiltering(
                                                        bilateral_filter_kernel_size_,
                                                        bilateral_filter_sigma_pos_,
                                                        bilateral_filter_sigma_value_,
                                                        bilateral_filter_iterations_);
                                                  return true;
                                                },
                                                [this]() { return bilateral_filter_iterations_; },
                                                "Filter iterations",
                                                "Depth map filter iteration count",
                                                bilateral_filter_iterations_,
                                                1,
                                                8);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "hole_filling_kernel_size",
      "filtering",
      [this](const int& val) {
        hole_filling_kernel_size_ = val;
        if (zcamera_)
          zcamera_->setHoleFiltering(hole_filling_kernel_size_, hole_filling_iterations_);
        return true;
      },
      [this]() { return hole_filling_kernel_size_; },
      "Hole filling kernel size",
      "Hole filling kernel size",
      hole_filling_kernel_size_,
      0,
      4);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "hole_filling_iterations",
      "filtering",
      [this](const int& val) {
        hole_filling_iterations_ = val;
        if (zcamera_)
          zcamera_->setHoleFiltering(hole_filling_kernel_size_, hole_filling_iterations_);
        return true;
      },
      [this]() { return hole_filling_iterations_; },
      "Hole filling iterations",
      "Hole filling iterations",
      hole_filling_iterations_,
      0,
      8);

  pmanage<MPtr(&PContainer::make_parented_bool)>(
      "filter_outliers",
      "filtering",
      [this](const bool& val) {
        if (filter_outliers_ != val && val == true) {
          filter_outliers_ = val;
          filter_mean_k_id_ = pmanage<MPtr(&PContainer::make_int)>(
              "filter_mean_k",
              [this](const int& val) {
                filter_mean_k_ = val;
                if (zcamera_)
                  zcamera_->setOutlierFilterParameters(
                      filter_outliers_, filter_mean_k_, filter_stddev_mul_);
                return true;
              },
              [this]() { return filter_mean_k_; },
              "Filter mean k",
              "Number of points to consider for the outlier filtering",
              filter_mean_k_,
              0,
              16);
          filter_stddev_mul_id_ = pmanage<MPtr(&PContainer::make_double)>(
              "filter_stddev_mul",
              [this](const double& val) {
                filter_stddev_mul_ = val;
                if (zcamera_)
                  zcamera_->setOutlierFilterParameters(
                      filter_outliers_, filter_mean_k_, filter_stddev_mul_);
                return true;
              },
              [this]() { return filter_stddev_mul_; },
              "Filter stddev mul",
              "Multiplier threshold on the statistical outlier filter (see PCL "
              "doc)",
              filter_stddev_mul_,
              0.0,
              8.0);
        } else if (filter_outliers_ != val && val == false) {
          filter_outliers_ = val;
          pmanage<MPtr(&PContainer::remove)>(filter_mean_k_id_);
          pmanage<MPtr(&PContainer::remove)>(filter_stddev_mul_id_);
        }
        if (zcamera_)
          zcamera_->setOutlierFilterParameters(
              filter_outliers_, filter_mean_k_, filter_stddev_mul_);
        return true;
      },
      [this]() { return filter_outliers_; },
      "Filter outliers",
      "Filter outlier points from the cloud",
      filter_outliers_);

  pmanage<MPtr(&PContainer::make_selection<>)>("capture_mode",
                                               [this](const IndexOrName& val) {
                                                 capture_modes_enum_.select(val);
                                                 return true;
                                               },
                                               [this]() { return capture_modes_enum_.get(); },
                                               "Capture mode",
                                               "Capture mode of the device",
                                               capture_modes_enum_);
}

PostureSrc::~PostureSrc() { stop(); }

bool PostureSrc::start() {
  if (random_data_) {
    random_data_thread_ = thread([this]() { generateRandomData(); });
    return true;
  } else {
    calibration_reader_->loadCalibration(calibration_path_);
    if (!(*calibration_reader_) ||
        calibration_reader_->getCalibrationParams().size() < device_index_)
      return false;

    zcamera_->setCalibration(calibration_reader_->getCalibrationParams()[device_index_]);
    zcamera_->setDeviceIndex(device_index_);
    zcamera_->setCaptureIR(capture_ir_);
    zcamera_->setBuildMesh(build_mesh_);
    zcamera_->setCompression(compress_cloud_);
    zcamera_->setCaptureMode((posture::ZCamera::CaptureMode)capture_modes_enum_.get());
    zcamera_->setOutlierFilterParameters(filter_outliers_, filter_mean_k_, filter_stddev_mul_);

    zcamera_->start();

    rgb_focal_ = zcamera_->getRGBFocal();
    depth_focal_ = zcamera_->getDepthFocal();

    rgb_focal_id_ = pmanage<MPtr(&PContainer::make_double)>("rgb_focal",
                                                            nullptr,
                                                            [this]() { return rgb_focal_; },
                                                            "RGB focal length",
                                                            "RGB focal length",
                                                            rgb_focal_,
                                                            0.0,
                                                            10000.0);

    depth_focal_id_ =
        pmanage<MPtr(&PContainer::make_double)>("depth_focal",
                                                [this](const double& val) {
                                                  zcamera_->setDepthFocal(val);
                                                  depth_focal_ = zcamera_->getDepthFocal();
                                                  return true;
                                                },
                                                [this]() {
                                                  depth_focal_ = zcamera_->getDepthFocal();
                                                  return depth_focal_;
                                                },
                                                "Depth focal length",
                                                "Depth focal length",
                                                depth_focal_,
                                                0.0,
                                                10000.0);

    return true;
  }
}

bool PostureSrc::stop() {
  if (random_data_) {
    do_random_data_ = false;
    if (random_data_thread_.joinable()) random_data_thread_.join();

    cloud_writer_.reset();
    mesh_writer_.reset();
  } else {
    if (zcamera_) zcamera_->stop();

    cloud_writer_.reset();
    mesh_writer_.reset();
    depth_writer_.reset();
    rgb_writer_.reset();
    ir_writer_.reset();

    pmanage<MPtr(&PContainer::remove)>(depth_focal_id_);
    pmanage<MPtr(&PContainer::remove)>(rgb_focal_id_);
  }

  return true;
}

void PostureSrc::cb_frame_cloud(void* context, const vector<char>&& data) {
  PostureSrc* ctx = (PostureSrc*)context;

  if (!ctx->cloud_writer_ ||
      data.size() > ctx->cloud_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
    auto data_type =
        ctx->compress_cloud_ ? string(POINTCLOUD_TYPE_COMPRESSED) : string(POINTCLOUD_TYPE_BASE);
    ctx->cloud_writer_.reset();
    ctx->cloud_writer_ = std::make_unique<ShmdataWriter>(
        ctx, ctx->make_file_name("cloud"), data.size() * 2, data_type);

    if (!ctx->cloud_writer_) {
      warning("Unable to create mesh callback");
      return;
    }
  }

  if (ctx->reload_calibration_) {
    ctx->calibration_reader_->loadCalibration(ctx->calibration_path_);
    if (!(*ctx->calibration_reader_) ||
        ctx->calibration_reader_->getCalibrationParams().size() < ctx->device_index_)
      return;
    ctx->zcamera_->setCalibration(
        ctx->calibration_reader_->getCalibrationParams()[ctx->device_index_]);
  }

  ctx->cloud_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(const_cast<char*>(data.data()),
                                                                  data.size());
  ctx->cloud_writer_->bytes_written(data.size());
}

void PostureSrc::cb_frame_mesh(void* context, vector<unsigned char>&& data) {
  PostureSrc* ctx = static_cast<PostureSrc*>(context);

  if (!ctx->mesh_writer_ ||
      data.size() > ctx->mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
    ctx->mesh_writer_.reset();
    ctx->mesh_writer_ = std::make_unique<ShmdataWriter>(
        ctx, ctx->make_file_name("mesh"), data.size() * 2, string(POLYGONMESH_TYPE_BASE));

    if (!ctx->mesh_writer_) {
      warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
      const_cast<unsigned char*>(data.data()), data.size());
  ctx->mesh_writer_->bytes_written(data.size());
}

void PostureSrc::cb_frame_depth(void* context,
                                const vector<unsigned char>& data,
                                int width,
                                int height) {
  PostureSrc* ctx = (PostureSrc*)context;

  if (!ctx->depth_writer_ || ctx->depth_width_ != width || ctx->depth_height_ != height) {
    ctx->depth_width_ = width;
    ctx->depth_height_ = height;
    char buffer[256] = "";
    sprintf(buffer,
            "video/x-raw, format=(string)GRAY16_BE, "
            "width=(int)%i, height=(int)%i, "
            "framerate=30/1, pixel-aspect-ratio=1/1",
            width,
            height);

    ctx->depth_writer_.reset();
    ctx->depth_writer_ = std::make_unique<ShmdataWriter>(
        ctx, ctx->make_file_name("depth"), data.size(), string(buffer));

    if (!ctx->depth_writer_) {
      ctx->warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->depth_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
      const_cast<unsigned char*>(data.data()), data.size());
  ctx->depth_writer_->bytes_written(data.size());
}

void PostureSrc::cb_frame_rgb(void* context,
                              const vector<unsigned char>& data,
                              int width,
                              int height) {
  using namespace std::placeholders;

  PostureSrc* ctx = (PostureSrc*)context;
  auto format = ctx->zcamera_->getCaptureFormat();

  if (!ctx->rgb_writer_ || ctx->rgb_width_ != width || ctx->rgb_height_ != height ||
      ctx->rgb_format_ != format) {
    ctx->rgb_width_ = width;
    ctx->rgb_height_ = height;
    ctx->rgb_format_ = format;

    string formatStr;
    switch (format) {
      default:
        formatStr = "RGB";
        break;
      case ZCamera::CaptureFormat::RGB:
        formatStr = "RGB";
        break;
      case ZCamera::CaptureFormat::BGR:
        formatStr = "BGR";
        break;
    }

    char buffer[256] = "";
    sprintf(buffer,
            "video/x-raw, format=(string)%s, width=(int)%i, "
            "height=(int)%i, framerate=30/1, pixel-aspect-ratio=1/1",
            formatStr.c_str(),
            width,
            height);

    ctx->rgb_writer_.reset();
    ctx->rgb_writer_ = std::make_unique<ShmdataWriter>(
        ctx, ctx->make_file_name("rgb"), data.size(), string(buffer));

    if (!ctx->rgb_writer_) {
      ctx->warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->rgb_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
      const_cast<unsigned char*>(data.data()), data.size());
  ctx->rgb_writer_->bytes_written(data.size());
}

void PostureSrc::cb_frame_ir(void* context,
                             const vector<unsigned char>& data,
                             int width,
                             int height) {
  PostureSrc* ctx = (PostureSrc*)context;

  if (!ctx->ir_writer_ || ctx->ir_width_ != width || ctx->ir_height_ != height) {
    ctx->ir_width_ = width;
    ctx->ir_height_ = height;

    char buffer[256] = "";
    sprintf(buffer,
            "video/x-raw, format=(string)GRAY16_BE, width=(int)%i, height=(int)%i, "
            "framerate=30/1, pixel-aspect-ratio=1/1",
            width,
            height);

    ctx->ir_writer_.reset();
    ctx->ir_writer_ = std::make_unique<ShmdataWriter>(
        ctx, ctx->make_file_name("ir"), data.size(), string(buffer));

    if (!ctx->ir_writer_) {
      ctx->warning("Unable to create mesh callback");
      return;
    }
  }

  ctx->ir_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
      const_cast<unsigned char*>(data.data()), data.size());
  ctx->ir_writer_->bytes_written(data.size());
}

void PostureSrc::generateRandomData() {
  do_random_data_ = true;

  while (do_random_data_) {
    // Random cloud
    vector<char> cloud;
    posture::ZCamera::getNoise(1, 1, 1, 1000, cloud);

    if (!cloud_writer_) {
      cloud_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("cloud"), cloud.size() * 2, string(POINTCLOUD_TYPE_BASE));
      if (!cloud_writer_) {
        ctx->warning("Unable to create mesh shmdata writer");
        return;
      }
    }

    cloud_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(const_cast<char*>(cloud.data()),
                                                               cloud.size());
    cloud_writer_->bytes_written(cloud.size());

    // Random mesh
    vector<unsigned char> mesh;
    posture::ZCamera::getRandomMesh(1, 1, 1, 100, mesh);

    if (!mesh_writer_) {
      mesh_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("mesh"), mesh.size() * 2, string(POLYGONMESH_TYPE_BASE));
      if (!mesh_writer_) {
        ctx->warning("Unable to create mesh shmdata writer");
        return;
      }
    }

    mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
        const_cast<unsigned char*>(mesh.data()), mesh.size());
    mesh_writer_->bytes_written(mesh.size());

    this_thread::sleep_for(chrono::milliseconds(33));
  }
}

}  // namespace switcher
