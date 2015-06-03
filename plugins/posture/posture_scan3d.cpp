#include "./posture_scan3d.hpp"
#include "switcher/std2.hpp"

#include <functional>
#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/io/obj_io.h>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSc3,
                                     "Scan 3D",
                                     "video",
                                     "Grabs meshes using zcameras",
                                     "LGPL",
                                     "posturescansrc",
                                     "Ludovic Schreiber");

PostureSc3::PostureSc3(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()){
    merger_ = std::unique_ptr<PointCloudMerger> (new PointCloudMerger());
    sol_ = std::unique_ptr<Solidify> (new Solidify());
    sol_->setGridResolution(50);
}

PostureSc3::~PostureSc3()
{
    stop();
}

bool PostureSc3::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (index_=0; index_<nbr_; index_++)
  {
    cameras_[index_]->start();
  }
  merger_->start();

  is_started_ = true;
  return true;
}

bool PostureSc3::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (index_=0; index_<nbr_; index_++)
  {
    cameras_[index_]->stop();
  }
  merger_->stop();

  mesh_writer_.reset();

  is_started_ = false;
  return true;
}

bool PostureSc3::init()
{
  init_startable(this);

  set_input_camera(nbr_, this);

  nbr_props_ = custom_props_->make_int_property("camera_number",
                            "Nuber of used cameras",
                            1,
                            9,
                            nbr_,
                            (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSc3::set_input_camera,
                            PostureSc3::get_input_camera,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            nbr_props_, "camera_number",
                            "Number of used cameras");

  calibration_path_prop_ =
      custom_props_->make_string_property("calibration_path",
                            "Path to the calibration file",
                            calibration_path_.c_str(),
                            (GParamFlags) G_PARAM_READWRITE,
                            PostureSc3::set_calibration_path,
                            PostureSc3::get_calibration_path,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            calibration_path_prop_, "calibration_path",
                            "Path to the calibration file");

  devices_path_prop_ = custom_props_->make_string_property("devices_path",
                            "Path to the devices description file",
                            devices_path_.c_str
                            (), (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSc3::set_devices_path,
                            PostureSc3::get_devices_path,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            devices_path_prop_, "devices",
                            "Path to the devices description file");

  grid_res_props_ = custom_props_->make_int_property("grid resolution",
                            "resolution for the mesh reconstruction",
                            3,
                            99,
                            grid_res_,
                            (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSc3::set_grid_props,
                            PostureSc3::get_grid_props,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            grid_res_props_, "grid resolution",
                            "resolution for the mesh reconstruction");

  reload_calibration_prop_ = custom_props_->make_boolean_property("reload_calibration",
                                "Reload calibration at each frame",
                                reload_calibration_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureSc3::set_reload_calibration,
                                PostureSc3::get_reload_calibration,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            reload_calibration_prop_, "reload_calibration",
                            "Reload calibration at each frame");

  return true;
}

int PostureSc3::get_input_camera(void* context)
{
  PostureSc3 *ctx = static_cast<PostureSc3*>(context);
  return ctx->nbr_;
}

void PostureSc3::set_input_camera(const int camera_nbr, void* user_data)
{
  PostureSc3 *ctx = static_cast<PostureSc3*>(user_data);

  ctx->nbr_ = camera_nbr;

  ctx->cameras_.resize(ctx->nbr_);
  ctx->merger_->setCloudNbr (ctx->nbr_);
  for (ctx->index_ = 0; ctx->index_ < ctx->nbr_; ctx->index_++)
  {
    std::shared_ptr<ZCamera> newCam = std::shared_ptr<ZCamera>(new ZCamera());
    ctx->cameras_[ctx->index_] = newCam;
    ctx->cameras_[ctx->index_]->setDeviceIndex(ctx->index_);
    ctx->cameras_[ctx->index_]->setCaptureMode(ZCamera::CaptureMode_QQVGA_30Hz);

    int index = ctx->index_;

    ctx->cameras_[ctx->index_]->setCallbackCloud([=](void* user_data, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) -> void {
        ctx->cb_frame_cloud(index, std::move(cloud));
    }, nullptr);
  }
}

const gchar *
PostureSc3::get_calibration_path(void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  return ctx->calibration_path_.c_str();
}

void
PostureSc3::set_calibration_path(const gchar *name, void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  if (name != nullptr)
  {
    ctx->calibration_path_ = name;
    ctx->merger_->setCalibrationPath(ctx->calibration_path_);
  }
}

const gchar *
PostureSc3::get_devices_path(void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  return ctx->devices_path_.c_str();
}

void
PostureSc3::set_devices_path(const gchar *name, void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  if (name != nullptr)
  {
    ctx->devices_path_ = name;
    ctx->merger_->setDevicesPath(ctx->devices_path_);
  }
}

void
PostureSc3::set_grid_props(const int resolution, void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  if (resolution >= 3)
  {
    ctx->grid_res_ = resolution;
    ctx->sol_->setGridResolution(ctx->grid_res_);
  }
}

int
PostureSc3::get_grid_props(void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  return ctx->grid_res_;
}

int
PostureSc3::get_reload_calibration(void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  return ctx->reload_calibration_;
}

void
PostureSc3::set_reload_calibration(const int reload, void *user_data) {
  PostureSc3 *ctx = (PostureSc3 *) user_data;
  ctx->reload_calibration_ = reload;
}

void PostureSc3::cb_frame_cloud(int index,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_started_)
      return;
  if (reload_calibration_)
      merger_->reloadCalibration();
  merger_->setInputCloud(index, cloud);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  merger_->getCloud(temp_cloud);

  sol_->setInputCloud(temp_cloud);
  sol_->getMesh(output_);

  if (!mesh_writer_ || output_.size() > mesh_writer_->writer(&shmdata::Writer::alloc_size)) {
      mesh_writer_.reset();
      if (output_.size()>= 256)
      {
        mesh_writer_ = std2::make_unique<ShmdataWriter>(this,
                                                         make_file_name("mesh"),
                                                         output_.size() * 2,
                                                         string(POLYGONMESH_TYPE_BASE));
      }
      else
      {
          mesh_writer_ = std2::make_unique<ShmdataWriter>(this,
                                                           make_file_name("mesh"),
                                                           512,
                                                           string(POLYGONMESH_TYPE_BASE));
      }

      if (!mesh_writer_) {
        g_warning("Unable to create mesh callback");
        return;
      }
  }

  mesh_writer_->writer(&shmdata::Writer::copy_to_shm, (void *) &output_, output_.size());
  mesh_writer_->bytes_written(output_.size());
}

}
