#include "./posture_scan3d.hpp"
#include "switcher/std2.hpp"

#include <functional>
#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/io/obj_io.h>

using namespace std;
using namespace
switcher::data;
using namespace
posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSc3,
                                     "Scan 3D",
                                     "video",
                                     "Grabs meshe using zcameras",
                                     "LGPL",
                                     "postures_scan_3d",
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

  return true;
}

bool PostureSc3::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  merger_->stop();
  for (index_=0; index_<nbr_; index_++)
  {
    cameras_[index_]->stop();
  }

  mesh_writer_.reset();

  return true;
}

bool PostureSc3::init()
{
  init_startable(this);

  nbr_props_ = custom_props_->make_int_property("camera_number",
                            "Nuber of used cameras",
                            0,
                            7,
                            nbr_,
                            (GParamFlags)
                            G_PARAM_READWRITE,
                            PostureSc3::set_input_camera,
                            PostureSc3::get_input_camera,
                            this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            nbr_props_, "camera_number",
                            "Nuber of used cameras");

  return true;
}

void PostureSc3::set_input_camera(const int camera_nbr, void* user_data)
{
  PostureSc3 *ctx = static_cast<PostureSc3*>(user_data);

  ctx->nbr_ = camera_nbr;

  ctx->cameras_.resize(ctx->nbr_);
  ctx->merger_->setCloudNbr (ctx->nbr_);
  for (ctx->index_=0; ctx->index_<ctx->nbr_; ctx->index_++)
  {
      std::shared_ptr<ZCamera> newCam = std::shared_ptr<ZCamera>(new ZCamera());
      ctx->cameras_[ctx->index_] = newCam;
      ctx->cameras_[ctx->index_]->setDeviceIndex(ctx->index_);
      ctx->cameras_[ctx->index_]->setCaptureMode(ZCamera::CaptureMode_QQVGA_30Hz);
      ctx->cameras_[ctx->index_]->setCallbackCloud(handle_cloud, user_data);
  }
}

int PostureSc3::get_input_camera(void* context)
{
  PostureSc3 *ctx = static_cast<PostureSc3*>(context);
  return ctx->nbr_;
}

void
PostureSc3::cb_frame_mesh(void* context, vector<unsigned char>&& data)
{
  PostureSc3 *ctx = static_cast<PostureSc3*>(context);

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

void PostureSc3::handle_cloud(void* context,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
  PostureSc3 *ctx = (PostureSc3*) context;

  std::lock_guard<std::mutex> lock(ctx->mutex_);

  ctx->merger_->setInputCloud(ctx->index_, cloud);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  ctx->merger_->getCloud(temp_cloud);
  ctx->sol_->setInputCloud(temp_cloud);

  ctx->sol_->getMesh(ctx->output_);
}

}
