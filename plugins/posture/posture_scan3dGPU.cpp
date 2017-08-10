#include "./posture_scan3dGPU.hpp"

#include <functional>
#include <iostream>

#include <pcl/io/obj_io.h>
#include <boost/make_shared.hpp>

#include "switcher/scope-exit.hpp"

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureScan3DGPU,
                                     "posturescan3dgpu",
                                     "RGBD to textured mesh",
                                     "video",
                                     "writer/device",
                                     "Create a textured mesh from rgbd cameras on GPU",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureScan3DGPU::~PostureScan3DGPU() {}

bool PostureScan3DGPU::start() {
  cameras_.clear();
  solidifyGPU_.reset();
  colorize_.reset();
  sculpt_.reset();
  mesh_writer_.reset();
  texture_writer_.reset();

  images_.resize(camera_nbr_);
  images_dims_.resize(camera_nbr_);
  cameras_updated_.resize(camera_nbr_);

  calibration_reader_->loadCalibration(calibration_path_);
  if (!(*calibration_reader_) ||
      calibration_reader_->getCalibrationParams().size() < (uint32_t)camera_nbr_)
    return false;

  for (int i = 0; i < camera_nbr_; ++i) {
    cameras_.emplace_back(new posture::ZCamera());
    cameras_.back()->setCaptureMode((posture::ZCamera::CaptureMode)capture_modes_enum_.get());
    cameras_.back()->setCalibration(calibration_reader_->getCalibrationParams()[i]);
    cameras_.back()->setDeviceIndex(i);

    cameras_.back()->setCallbackCloud(
        [=](void*, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
          cb_frame_cloud(i, cloud);
        },
        nullptr);

    cameras_.back()->setCallbackDepth(
        [=](void*, std::vector<uint8_t>& depth, int width, int height) {
          cb_frame_depth(i, depth, width, height);
        },
        nullptr);

    cameras_.back()->setCallbackRgb(
        [=](void*, std::vector<uint8_t>& rgb, int width, int height) {
          cb_frame_RGB(i, rgb, width, height);
        },
        nullptr);
  }

  reset_solidify();

  if (refine_mesh_) {
    sculpt_ = unique_ptr<posture::SculptGL>(new posture::SculptGL());
    sculpt_->setPointSeparation(refine_mesh_point_separation_);
    sculpt_->setMaxNeighbors(refine_mesh_max_neighbours_);
  }

  colorize_ = unique_ptr<posture::ColorizeGL>(new posture::ColorizeGL());
  colorize_->setCalibration(calibration_reader_->getCalibrationParams());
  colorize_->setCompressMesh(compress_mesh_);

  update_loop_started_ = true;
  update_thread_ = thread([&]() { update_loop(); });

  return true;
}

bool PostureScan3DGPU::stop() {
  update_loop_started_ = false;
  unique_lock<mutex> lockUpdate(update_mutex_);
  update_cv_.notify_one();
  if (update_thread_.joinable()) update_thread_.join();

  if (registering_thread_.joinable()) registering_thread_.join();

  cameras_.clear();
  solidifyGPU_.reset();
  colorize_.reset();
  mesh_writer_.reset();
  texture_writer_.reset();

  return true;
}

void PostureScan3DGPU::update_loop() {
  for (auto& cam : cameras_) {
    cam->start();
    if (!cam->isReady()) return;
  }

  std::vector<pcl::PolygonMesh::Ptr> multimesh;
  pcl::PolygonMesh::Ptr mesh = boost::make_shared<pcl::PolygonMesh>();

  pcl::TextureMesh::Ptr texturedMesh = boost::make_shared<pcl::TextureMesh>();
  std::vector<uint8_t> mesh_serialized{};

  while (update_loop_started_) {
    unique_lock<mutex> lock(update_mutex_);
    if (!update_wanted_) update_cv_.wait(lock);

    update_wanted_ = false;

    if (!update_loop_started_) break;

    // The registerer runs in a separate thread and is updated at
    // its own pace
    if (improve_registering_ && !is_registering_) {
      if (registering_thread_.joinable()) registering_thread_.join();

      is_registering_ = true;
      registering_thread_ = thread([=]() {
        calibration_reader_->reload();
        if (*calibration_reader_ &&
            calibration_reader_->getCalibrationParams().size() >= (uint32_t)camera_nbr_) {
          auto calibration = calibration_reader_->getCalibrationParams();

          register_->setGuessCalibration(calibration);
          calibration = register_->getCalibration();

          // The previous call can take some time, so we check again that
          // automatic registration is active
          if (improve_registering_) {
            solidifyGPU_->setCalibration(calibration);
            colorize_->setCalibration(calibration);
          }
        }

        is_registering_ = false;
        pmanage<MPtr(&PContainer::set<bool>)>(register_id_, false);
      });
    }

    if (reload_calibration_) {
      calibration_reader_->reload();
      if (*calibration_reader_ &&
          calibration_reader_->getCalibrationParams().size() >= (uint32_t)camera_nbr_) {
        auto calibration = calibration_reader_->getCalibrationParams();
        solidifyGPU_->setCalibration(calibration);
        colorize_->setCalibration(calibration);
      }
    }

    // get geometry from Marching Cubes running on the depth images
    if (compress_multicore_)
      solidifyGPU_->getMesh(multimesh);
    else
      solidifyGPU_->getMesh(mesh);

    // sculpt not working on multimeshes
    if (refine_mesh_ && !compress_multicore_) {
      sculpt_->setInputMesh(mesh);
      sculpt_->getMesh(mesh);
    }

    lock.unlock();

    unique_lock<mutex> lockCamera(camera_mutex_);
    if (compress_multicore_)
      colorize_->setInput(multimesh, images_, images_dims_);
    else
      colorize_->setInput(mesh, images_, images_dims_);
    lockCamera.unlock();
    colorize_->getTexturedMesh(mesh_serialized);

    // retrieve the merged image and its dimensions
    uint32_t width, height;
    auto texture = colorize_->getTexture(width, height);

    // write the serialization to a shmdata
    if (mesh_serialized.size() > 0) {
      if (!mesh_writer_ ||
          mesh_serialized.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
        string data_type = compress_multicore_ ? POLYGONMESH_TYPE_MULTIMESH : POLYGONMESH_TYPE_BASE;
        mesh_writer_.reset();
        mesh_writer_ = std::make_unique<ShmdataWriter>(
            this, make_file_name("mesh"), mesh_serialized.size() * 2, data_type);

        if (!mesh_writer_) {
          warning("Unable to create mesh writer");
          return;
        }
      }

      mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
          reinterpret_cast<char*>(mesh_serialized.data()), mesh_serialized.size());
      mesh_writer_->bytes_written(mesh_serialized.size());

      if (!texture_writer_ ||
          texture.size() > texture_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
        texture_writer_.reset();
        texture_writer_ = std::make_unique<ShmdataWriter>(
            this,
            make_file_name("texture"),
            texture.size() * 2,
            "video/x-raw,format=(string)RGB,width=(int)" + to_string(width) + ",height=(int)" +
                to_string(height) + ",framerate=30/1");

        if (!texture_writer_) {
          warning("Unable to create texture writer");
          return;
        }
      }

      texture_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
          reinterpret_cast<char*>(texture.data()), texture.size());
      texture_writer_->bytes_written(texture.size());
    }
  }  // while(update_loop_started_)
}  // update_loop()

PostureScan3DGPU::PostureScan3DGPU(QuiddityConfiguration&&) {
  calibration_reader_ = std::make_unique<CalibrationReader>("default.kvc");
  register_ = std::make_unique<Register>();

  init_startable(this);

  pmanage<MPtr(&PContainer::make_selection<>)>("capture_mode",
                                               [this](const IndexOrName& val) {
                                                 capture_modes_enum_.select(val);
                                                 return true;
                                               },
                                               [this]() { return capture_modes_enum_.get(); },
                                               "Capture mode",
                                               "Capture mode for the cameras",
                                               capture_modes_enum_);

  pmanage<MPtr(&PContainer::make_int)>("camera_nbr",
                                       [this](const int& val) {
                                         camera_nbr_ = std::max(1, val);
                                         return true;
                                       },
                                       [this]() { return camera_nbr_; },
                                       "Number of cameras",
                                       "Number of cameras to grab from",
                                       camera_nbr_,
                                       1,
                                       7);

  pmanage<MPtr(&PContainer::make_bool)>("compress_mesh",
                                        [this](const bool& val) {
                                          compress_mesh_ = val;
                                          if (solidifyGPU_)
                                            colorize_->setCompressMesh(compress_mesh_);
                                          return true;
                                        },
                                        [this]() { return compress_mesh_; },
                                        "Compress mesh",
                                        "Compress the generated mesh",
                                        compress_mesh_);

  pmanage<MPtr(&PContainer::make_bool)>("compress_multicore",
                                        [this](const bool& val) {
                                          compress_multicore_ = val;
                                          return true;
                                        },
                                        [this]() { return compress_multicore_; },
                                        "Multithreaded compression",
                                        "Compress on multiple cores if true",
                                        compress_multicore_);

  //
  // Calibration
  pmanage<MPtr(&PContainer::make_group)>(
      "calibration", "Calibration", "Camera calibration parameters");

  pmanage<MPtr(&PContainer::make_parented_string)>("calibration_path",
                                                   "calibration",
                                                   [this](const std::string& val) {
                                                     calibration_path_ = val;
                                                     return true;
                                                   },
                                                   [this]() { return calibration_path_; },
                                                   "Calibration path",
                                                   "Path to the calibration file",
                                                   calibration_path_);

  register_id_ =
      pmanage<MPtr(&PContainer::make_parented_bool)>("reload_calibration",
                                                     "calibration",
                                                     [this](const bool& val) {
                                                       reload_calibration_ = val;
                                                       return true;
                                                     },
                                                     [this]() { return reload_calibration_; },
                                                     "Reload calibration",
                                                     "Reload the calibration from the given file",
                                                     reload_calibration_);

  register_id_ = pmanage<MPtr(&PContainer::make_parented_bool)>(
      "improve_registration",
      "calibration",
      [this](const bool& val) {
        improve_registering_ = val;
        return true;
      },
      [this]() { return improve_registering_; },
      "Improve registration",
      "Automatically improve cameras calibration, once",
      improve_registering_);

  //
  // Grid parameters
  pmanage<MPtr(&PContainer::make_group)>(
      "grid", "Grid parameters", "Reconstruction grid parameters");

  pmanage<MPtr(&PContainer::make_parented_double)>(
      "grid_size",
      "grid",
      [this](const double& val) {
        grid_size_[0] = val;
        grid_size_[1] = val;
        grid_size_[2] = val;
        if (solidifyGPU_) reset_solidify();

        pmanage<MPtr(&PContainer::notify)>(grid_size_x_id_);
        pmanage<MPtr(&PContainer::notify)>(grid_size_y_id_);
        pmanage<MPtr(&PContainer::notify)>(grid_size_z_id_);

        return true;
      },
      [this]() { return std::max(grid_size_[0], std::max(grid_size_[1], grid_size_[2])); },
      "Mesh grid size",
      "Size in meters of the grid enclosing the mesh",
      grid_size_[0],
      0.2,
      10.0);

  grid_size_x_id_ = pmanage<MPtr(&PContainer::make_parented_double)>(
      "grid_size_x",
      "grid",
      [this](const double& val) {
        grid_size_[0] = val;
        if (solidifyGPU_) reset_solidify();
        return true;
      },
      [this]() { return grid_size_[0]; },
      "Mesh grid size along X",
      "Size in meters of the grid along the X axis",
      grid_size_[0],
      0.2,
      10.0);

  grid_size_y_id_ = pmanage<MPtr(&PContainer::make_parented_double)>(
      "grid_size_y",
      "grid",
      [this](const double& val) {
        grid_size_[1] = val;
        if (solidifyGPU_) reset_solidify();
        return true;
      },
      [this]() { return grid_size_[1]; },
      "Mesh grid size along Y",
      "Size in meters of the grid along the Y axis",
      grid_size_[1],
      0.2,
      10.0);

  grid_size_z_id_ = pmanage<MPtr(&PContainer::make_parented_double)>(
      "grid_size_z",
      "grid",
      [this](const double& val) {
        grid_size_[2] = val;
        if (solidifyGPU_) reset_solidify();
        return true;
      },
      [this]() { return grid_size_[2]; },
      "Mesh grid size along Z",
      "Size in meters of the grid along the Z axis",
      grid_size_[2],
      0.2,
      10.0);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "grid_resolution",
      "grid",
      [this](const int& val) {
        grid_resolution_ = val;
        if (solidifyGPU_) reset_solidify();
        return true;
      },
      [this]() { return grid_resolution_; },
      "Mesh grid resolution",
      "Resolution of the grid enclosing the mesh, along the larger axis",
      grid_resolution_,
      16,
      256);

  //
  // Filtering
  pmanage<MPtr(&PContainer::make_group)>("filtering", "Filtering", "Filtering");

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "kernel_filter_size",
      "filtering",
      [this](const int& val) {
        kernel_filter_size_ = val;
        if (solidifyGPU_)
          solidifyGPU_->setDepthFiltering(
              kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
        return true;
      },
      [this]() { return kernel_filter_size_; },
      "Filter kernel size",
      "Depth map filter kernel size",
      kernel_filter_size_,
      1,
      15);

  pmanage<MPtr(&PContainer::make_parented_float)>(
      "kernel_spatial_sigma_",
      "filtering",
      [this](const float& val) {
        kernel_spatial_sigma_ = val;
        if (solidifyGPU_)
          solidifyGPU_->setDepthFiltering(
              kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
        return true;
      },
      [this]() { return kernel_spatial_sigma_; },
      "Filter spatial sigma",
      "Depth map filter spatial sigma",
      kernel_spatial_sigma_,
      0.1,
      16.0);

  pmanage<MPtr(&PContainer::make_parented_float)>(
      "kernel_value_sigma_",
      "filtering",
      [this](const float& val) {
        kernel_value_sigma_ = val;
        if (solidifyGPU_)
          solidifyGPU_->setDepthFiltering(
              kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
        return true;
      },
      [this]() { return kernel_value_sigma_; },
      "Filter value sigma",
      "Depth map filter value sigma",
      kernel_value_sigma_,
      1.0,
      1600.0);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "hole_filling_iterations",
      "filtering",
      [this](const int& val) {
        hole_filling_iterations_ = val;
        if (solidifyGPU_) solidifyGPU_->setHoleFillingIterations(val);
        return true;
      },
      [this]() { return hole_filling_iterations_; },
      "Hole filling iterations",
      "Number of iterations for the hole filling algorithm",
      hole_filling_iterations_,
      0,
      15);

  //
  // Refine mesh
  pmanage<MPtr(&PContainer::make_group)>("refine_mesh", "Refine mesh", "Refine mesh");

  register_id_ =
      pmanage<MPtr(&PContainer::make_parented_bool)>("active_refine_mesh",
                                                     "refine_mesh",
                                                     [this](const bool& val) {
                                                       refine_mesh_ = val;
                                                       return true;
                                                     },
                                                     [this]() { return refine_mesh_; },
                                                     "Refine mesh",
                                                     "Add details where point density is high",
                                                     refine_mesh_);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "refine_mesh_max_neighbours",
      "refine_mesh",
      [this](const int& val) {
        refine_mesh_max_neighbours_ = val;
        return true;
      },
      [this]() { return refine_mesh_max_neighbours_; },
      "Max neighbours",
      "Maximum number of points to consider for refinement",
      refine_mesh_max_neighbours_,
      1,
      32);

  pmanage<MPtr(&PContainer::make_parented_float)>(
      "refine_mesh_point_separation",
      "refine_mesh",
      [this](const float& val) {
        refine_mesh_point_separation_ = val;
        return true;
      },
      [this]() { return refine_mesh_point_separation_; },
      "Point separation",
      "Point separation of the cloud used for refinement",
      refine_mesh_point_separation_,
      0.0,
      1.0);
}

void PostureScan3DGPU::reset_solidify() {
  unique_lock<mutex> lock(update_mutex_);
  solidifyGPU_ = unique_ptr<posture::SolidifyGPU>(new posture::SolidifyGPU());
  solidifyGPU_->setCalibration(calibration_reader_->getCalibrationParams());
  solidifyGPU_->setGridSizeX(grid_size_[0]);
  solidifyGPU_->setGridSizeY(grid_size_[1]);
  solidifyGPU_->setGridSizeZ(grid_size_[2]);
  solidifyGPU_->setGridResolution(std::max(std::max(grid_size_[0], grid_size_[1]), grid_size_[2]) /
                                  (double)grid_resolution_);
  solidifyGPU_->setDepthMapNbr(camera_nbr_);
  solidifyGPU_->setDepthFiltering(kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
}

bool PostureScan3DGPU::all(const vector<bool>& status) {
  for (auto s : status)
    if (!s) return false;

  return true;
}

void PostureScan3DGPU::zero(vector<bool>& status) {
  for (uint32_t i = 0; i < status.size(); ++i) status[i] = false;
}

void PostureScan3DGPU::cb_frame_cloud(int index,
                                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
  unique_lock<mutex> lockCamera(camera_mutex_);
  register_->setInputCloud(index, cloud);
}

void PostureScan3DGPU::cb_frame_depth(int index,
                                      std::vector<unsigned char>& depth,
                                      int width,
                                      int height) {
  unique_lock<mutex> lockCamera(update_mutex_, std::try_to_lock);
  if (lockCamera.owns_lock()) {
    solidifyGPU_->setInputDepthMap(index, depth, width, height);
    bool already_updated = cameras_updated_[index];
    cameras_updated_[index] = true;
    if (already_updated || all(cameras_updated_)) {
      zero(cameras_updated_);
      update_wanted_ = true;
      update_cv_.notify_one();
    }
  }
}

void PostureScan3DGPU::cb_frame_RGB(int index,
                                    std::vector<unsigned char>& rgb,
                                    int width,
                                    int height) {
  unique_lock<mutex> lock(camera_mutex_);
  images_[index] = rgb;
  images_dims_[index] = {(uint32_t)width, (uint32_t)height, 3u};
}

}  // namespace switcher
