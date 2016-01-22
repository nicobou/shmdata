#include "./posture_scan3dGPU.hpp"
#include "switcher/std2.hpp"

#include <functional>
#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/io/obj_io.h>
#include "switcher/scope-exit.hpp"

using namespace std;
using namespace posture;

namespace switcher {
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION (
					PostureScan3DGPU,
          "posturesolidifygpu",
					"RGBD to textured mesh",
					"video",
          "writer/device",
					"Create a textured mesh from rgbd cameras on GPU",
					"LGPL",
					"Emmanuel Durand");

  PostureScan3DGPU::PostureScan3DGPU(const std::string &) {
  }

  PostureScan3DGPU::~PostureScan3DGPU() {
  }

  bool
  PostureScan3DGPU::start() {
    cameras_.clear();
    solidifyGPU_.reset();
    colorize_.reset();
    mesh_writer_.reset();
    texture_writer_.reset();

    images_.resize(camera_nbr_);
    images_dims_.resize(camera_nbr_);
    cameras_updated_.resize(camera_nbr_);

    for (int i = 0; i < camera_nbr_; ++i)
    {
      cameras_.emplace_back(new posture::ZCamera());
      cameras_.back()->setCaptureMode((posture::ZCamera::CaptureMode)capture_modes_enum_.get());
      cameras_.back()->setCalibrationPath(calibration_path_);
      cameras_.back()->setDeviceIndex(i);

      cameras_.back()->setCallbackDepth([=](void*, std::vector<uint8_t>& depth, int width, int height) {
        cb_frame_depth(i, depth, width, height);
      }, nullptr);

      cameras_.back()->setCallbackRgb([=](void*, std::vector<uint8_t>& rgb, int width, int height) {
        cb_frame_RGB(i, rgb, width, height);
      }, nullptr);
    }

    reset_solidify();

    colorize_ = unique_ptr<posture::ColorizeGL>(new posture::ColorizeGL());
    colorize_->setCalibrationPath(calibration_path_);

    update_loop_started_ = true;
    update_thread_ = thread([&](){
      update_loop();
    });

    return true;
  }

  bool
  PostureScan3DGPU::stop() {
    update_loop_started_ = false;
    unique_lock<mutex> lockUpdate(update_mutex_);
    update_cv_.notify_one();
    if (update_thread_.joinable())
      update_thread_.join();

    cameras_.clear();
    solidifyGPU_.reset();
    colorize_.reset();
    mesh_writer_.reset();
    texture_writer_.reset();

    return true;
  }

  void
  PostureScan3DGPU::update_loop() {
    for (auto& cam : cameras_) {
      cam->start();
      if (!cam->isReady())
        return;
    }

    pcl::PolygonMesh::Ptr mesh = boost::make_shared<pcl::PolygonMesh>();
    pcl::TextureMesh::Ptr texturedMesh = boost::make_shared<pcl::TextureMesh>();
    std::vector<uint8_t> mesh_serialized {};

    while(update_loop_started_) {
      unique_lock<mutex> lock(update_mutex_);
      if (!update_wanted_)
        update_cv_.wait(lock);

      update_wanted_ = false;

      if (!update_loop_started_)
        break;

      solidifyGPU_->getMesh(mesh);
      lock.unlock();

      unique_lock<mutex> lockCamera(camera_mutex_);
      colorize_->setInput(mesh, images_, images_dims_);
      lockCamera.unlock();

      colorize_->getTexturedMesh(texturedMesh);
      colorize_->getTexturedMesh(mesh_serialized);
      uint32_t width, height;
      auto texture = colorize_->getTexture(width, height);

      if (texturedMesh->tex_polygons.size() > 0) {
        if (!mesh_writer_ || mesh_serialized.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
          auto data_type = string(POLYGONMESH_TYPE_BASE);
          mesh_writer_.reset();
          mesh_writer_ = std2::make_unique<ShmdataWriter>(this,
                            make_file_name("mesh"),
                            mesh_serialized.size() * 2,
                            data_type);

          if (!mesh_writer_) {
            g_warning("Unable to create mesh writer");
            return;
          }
        }

        mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(reinterpret_cast<char*>(mesh_serialized.data()), mesh_serialized.size());
        mesh_writer_->bytes_written(mesh_serialized.size());

        if (!texture_writer_ || texture.size() > texture_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
          auto data_type = string(POINTCLOUD_TYPE_BASE);
          texture_writer_.reset();
          texture_writer_ = std2::make_unique<ShmdataWriter>(this,
                            make_file_name("texture"),
                            texture.size() * 2,
                            "video/x-raw,format=(string)BGR,width=(int)" + to_string(width) + ",height=(int)" + to_string(height) + ",framerate=30/1");

          if (!texture_writer_) {
            g_warning("Unable to create texture writer");
            return;
          }
        }

        texture_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(reinterpret_cast<char*>(texture.data()), texture.size());
        texture_writer_->bytes_written(texture.size());
      }
    }
  }

  bool PostureScan3DGPU::init() {
    init_startable(this);
    
    pmanage<MPtr(&PContainer::make_string)>(
      "calibration_path",
      [this](const std::string &val){calibration_path_ = val; return true;},
      [this](){return calibration_path_;},
      "Calibration path",
      "Path to the calibration file",
      calibration_path_);

    pmanage<MPtr(&PContainer::make_string)>(
      "devices_path",
      [this](const std::string &val){devices_path_ = val; return true;},
      [this](){return devices_path_;},
      "Devices path",
      "Path to the devices description file",
      devices_path_);

    pmanage<MPtr(&PContainer::make_selection)>(
      "capture_mode",
      [this](const size_t &val){capture_modes_enum_.select(val); return true;},
      [this](){return capture_modes_enum_.get();},
      "Capture mode",
      "Capture mode of the device",
      capture_modes_enum_);

    pmanage<MPtr(&PContainer::make_int)>(
      "camera_nbr",
      [this](const int &val){camera_nbr_ = std::max(1, val); return true;},
      [this](){return camera_nbr_;},
      "Cameras number",
      "Number of cameras to use",
      camera_nbr_,
      1,
      7);

    pmanage<MPtr(&PContainer::make_double)>(
      "grid_size",
      [this](const double &val){
        grid_size_[0] = val;
        grid_size_[1] = val;
        grid_size_[2] = val;
        if (solidifyGPU_)
          reset_solidify();
        return true;
      },
      [this](){return std::max(grid_size_[0], std::max(grid_size_[1], grid_size_[2]));},
      "Mesh grid size",
      "Size in meters of the grid enclosing the mesh",
      grid_size_[0],
      0.2,
      10.0);

    pmanage<MPtr(&PContainer::make_double)>(
      "grid_size_x",
      [this](const double &val){
        grid_size_[0] = val;
        if (solidifyGPU_)
          reset_solidify();
        return true;
      },
      [this](){return grid_size_[0];},
      "Mesh grid size along X",
      "Size in meters of the grid along the X axis",
      grid_size_[0],
      0.2,
      10.0);

    pmanage<MPtr(&PContainer::make_double)>(
      "grid_size_y",
      [this](const double &val){
        grid_size_[1] = val;
        if (solidifyGPU_)
          reset_solidify();
        return true;
      },
      [this](){return grid_size_[1];},
      "Mesh grid size along Y",
      "Size in meters of the grid along the Y axis",
      grid_size_[1],
      0.2,
      10.0);

    pmanage<MPtr(&PContainer::make_double)>(
      "grid_size_z",
      [this](const double &val){
        grid_size_[2] = val;
        if (solidifyGPU_)
          reset_solidify();
        return true;
      },
      [this](){return grid_size_[2];},
      "Mesh grid size along Z",
      "Size in meters of the grid along the Z axis",
      grid_size_[2],
      0.2,
      10.0);

    pmanage<MPtr(&PContainer::make_int)>(
      "grid_resolution",
      [this](const int &val){
        grid_resolution_ = val;
        if (solidifyGPU_)
          reset_solidify();
        return true;
      },
      [this](){return grid_resolution_;},
      "Mesh grid resolution",
      "Resolution of the grid enclosing the mesh, along the larger axis",
      grid_resolution_,
      16,
      256);

    pmanage<MPtr(&PContainer::make_int)>(
      "kernel_filter_size",
      [this](const int &val){
        kernel_filter_size_ = val;
        if (solidifyGPU_)
          solidifyGPU_->setDepthFiltering(kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
        return true;
      },
      [this](){return kernel_filter_size_;},
      "Filter kernel size",
      "Depth map filter kernel size",
      kernel_filter_size_,
      1,
      15);

    pmanage<MPtr(&PContainer::make_float)>(
      "kernel_spatial_sigma_",
      [this](const float &val){
        kernel_spatial_sigma_ = val;
        if (solidifyGPU_)
          solidifyGPU_->setDepthFiltering(kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
        return true;
      },
      [this](){return kernel_spatial_sigma_;},
      "Filter spatial sigma",
      "Depth map filter spatial sigma",
      kernel_spatial_sigma_,
      0.1,
      16.0);

    pmanage<MPtr(&PContainer::make_float)>(
      "kernel_value_sigma_",
      [this](const float &val){
        kernel_value_sigma_ = val;
        if (solidifyGPU_)
          solidifyGPU_->setDepthFiltering(kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
        return true;
      },
      [this](){return kernel_value_sigma_;},
      "Filter value sigma",
      "Depth map filter value sigma",
      kernel_value_sigma_,
      1.0,
      16000.0);

    return true;
  }

  void
  PostureScan3DGPU::reset_solidify()
  {
    unique_lock<mutex> lock(update_mutex_);
    solidifyGPU_ = unique_ptr<posture::SolidifyGPU>(new posture::SolidifyGPU());
    solidifyGPU_->setCalibrationPath(calibration_path_);
    solidifyGPU_->setGridSizeX(grid_size_[0]);
    solidifyGPU_->setGridSizeY(grid_size_[1]);
    solidifyGPU_->setGridSizeZ(grid_size_[2]);
    solidifyGPU_->setGridResolution(std::max(std::max(grid_size_[0], grid_size_[1]), grid_size_[2]) / (double)grid_resolution_);
    solidifyGPU_->setDepthMapNbr(camera_nbr_);
    solidifyGPU_->setDepthFiltering(kernel_filter_size_, kernel_spatial_sigma_, kernel_value_sigma_);
  }

  bool
  PostureScan3DGPU::all(const vector<bool>& status)
  {
    for (auto s : status)
      if (!s)
        return false;

    return true;
  }

  void
  PostureScan3DGPU::zero(vector<bool>& status)
  {
    for (uint32_t i = 0; i < status.size(); ++i)
      status[i] = false;
  }

  void
  PostureScan3DGPU::cb_frame_depth(int index, std::vector<unsigned char>& depth, int width, int height) {

    unique_lock<mutex> lockCamera(update_mutex_, std::try_to_lock);
    if (lockCamera.owns_lock()) {
      solidifyGPU_->setInputDepthMap(index, depth, width, height);
      bool already_updated = cameras_updated_[index];
      cameras_updated_[index] = true;
      if (already_updated || all(cameras_updated_))
      {
        zero(cameras_updated_);
        update_wanted_ = true;
        update_cv_.notify_one();
      }
    }
  }

  void
  PostureScan3DGPU::cb_frame_RGB(int index, std::vector<unsigned char>& rgb, int width, int height) {
    unique_lock<mutex> lock(camera_mutex_);
    images_[index] = rgb;
    images_dims_[index] = {(uint32_t)width, (uint32_t)height, 3u};
  }

}  // namespace switcher
