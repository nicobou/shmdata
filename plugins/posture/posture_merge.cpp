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

#include "./posture_merge.hpp"

#include <iostream>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureMerge,
                                     "Point Clouds Merge",
                                     "video",
                                     "Merges point clouds captured with 3D cameras",
                                     "LGPL",
                                     "pclmergesink", "Emmanuel Durand");

PostureMerge::PostureMerge():
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

PostureMerge::~PostureMerge() {
  stop();
}

bool
PostureMerge::start() {
  merger_ = make_shared<PointCloudMerger>("", source_id_);

  merger_->setCalibrationPath(calibration_path_);
  merger_->setDevicesPath(devices_path_);
  merger_->setCompression(compress_cloud_);
  merger_->setSaveCloud(save_cloud_);

  merger_->start();

  return true;
}

bool
PostureMerge::stop() {
  lock_guard<mutex> lock(mutex_);

  if (merger_.get() != nullptr) {
    merger_->stop();
    merger_.reset();
  }

  if (cloud_writer_.get() != nullptr) {
    unregister_shmdata(cloud_writer_->get_path());
    cloud_writer_.reset();
  }

  return true;
}

bool
PostureMerge::init() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&PostureMerge::connect, this, std::placeholders::_1), nullptr,     // FIXME implement this (disconnect with the shmdata as unique argument)
                         std::bind(&PostureMerge::disconnect_all, this),
                         std::bind(&PostureMerge::can_sink_caps, this, std::placeholders::_1),
                         8);

  calibration_path_prop_ =
      custom_props_->make_string_property("calibration_path",
                                          "Path to the calibration file",
                                          calibration_path_.c_str(),
                                          (GParamFlags) G_PARAM_READWRITE,
                                          PostureMerge::set_calibration_path,
                                          PostureMerge::get_calibration_path,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            calibration_path_prop_, "calibration_path",
                            "Path to the calibration file");

  devices_path_prop_ = custom_props_->make_string_property("devices_path",
                                                           "Path to the devices description file",
                                                           devices_path_.c_str
                                                           (), (GParamFlags)
                                                           G_PARAM_READWRITE,
                                                           PostureMerge::set_devices_path,
                                                           PostureMerge::get_devices_path,
                                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            devices_path_prop_, "devices",
                            "Path to the devices description file");

  compress_cloud_prop_ = custom_props_->make_boolean_property("compress_cloud",
                                           "Compress the cloud if true",
                                           compress_cloud_,
                                           (GParamFlags) G_PARAM_READWRITE,
                                           PostureMerge::set_compress_cloud,
                                           PostureMerge::get_compress_cloud,
                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            compress_cloud_prop_, "compress_cloud",
                            "Compress the cloud if true");

  reload_calibration_prop_ = custom_props_->make_boolean_property("reload_calibration",
                                "Reload calibration at each frame",
                                reload_calibration_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureMerge::set_reload_calibration,
                                PostureMerge::get_reload_calibration,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            reload_calibration_prop_, "reload_calibration",
                            "Reload calibration at each frame");

  save_cloud_prop_ = custom_props_->make_boolean_property("save_cloud",
                                           "Save the current cloud if true",
                                           save_cloud_,
                                           (GParamFlags) G_PARAM_READWRITE,
                                           PostureMerge::set_save_cloud,
                                           PostureMerge::get_save_cloud,
                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            save_cloud_prop_, "save_cloud",
                            "Save the current cloud if true");

  downsample_prop_ = custom_props_->make_boolean_property("downsample",
                                "Activate the cloud downsampling",
                                downsample_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureMerge::set_downsample_active,
                                PostureMerge::get_downsample_active,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            downsample_prop_, "downsample",
                            "Activate the cloud downsampling");

  return true;
}

bool
PostureMerge::connect(std::string shmdata_socket_path) {
  int index = source_id_;
  source_id_ += 1;

  ShmdataAnyReader::ptr reader_ = make_shared<ShmdataAnyReader>();
  reader_->set_path(shmdata_socket_path);

  // This is the callback for when new clouds are received
  reader_->set_callback([=] (void *data,
                             int size,
                             unsigned long long timestamp,
                             const char *type,
                             void * /*unused */ )
  {
    if (merger_ == nullptr || (string(type) != string(POINTCLOUD_TYPE_BASE) && string(type) != string(POINTCLOUD_TYPE_COMPRESSED)))
      return;

    if (reload_calibration_)
        merger_->reloadCalibration();

    // Setting input clouds is thread safe, so lets do it
    merger_->setInputCloud(index,
                           vector<char>((char*)data, (char*) data + size),
                           string(type) != string(POINTCLOUD_TYPE_BASE),
                           timestamp);

    // If another thread is trying to get the merged cloud, don't bother
    if (!mutex_.try_lock())
      return;

    if (cloud_writer_.get() == nullptr) {
      cloud_writer_.reset(new ShmdataAnyWriter);
      cloud_writer_->set_path(make_file_name("cloud"));
      register_shmdata(cloud_writer_);
      if (compress_cloud_)
        cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_COMPRESSED));
      else
        cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_BASE));
      cloud_writer_->start();
    }

    check_buffers();
    vector<char> cloud = merger_->getCloud();
    shmwriter_queue_.push_back(make_shared<vector<unsigned char>>(reinterpret_cast<const unsigned char*>(cloud.data()),
                                                                  reinterpret_cast<const unsigned char*>(cloud.data()) + cloud.size()));
    cloud_writer_->push_data_auto_clock((void *) shmwriter_queue_[shmwriter_queue_.size() - 1]->data(),
                                        cloud.size(),
                                        PostureMerge::free_sent_buffer,
                                        (void*)(shmwriter_queue_[shmwriter_queue_.size() - 1].get()));

    mutex_.unlock();
  },
  nullptr);

  reader_->start();
  register_shmdata(reader_);
  return true;
}

bool
PostureMerge::disconnect_all() {
  source_id_ = 0;
  return true;
}

const gchar *
PostureMerge::get_calibration_path(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->calibration_path_.c_str();
}

void
PostureMerge::set_calibration_path(const gchar *name, void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  if (name != nullptr)
    ctx->calibration_path_ = name;
}

const gchar *
PostureMerge::get_devices_path(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->devices_path_.c_str();
}

void
PostureMerge::set_devices_path(const gchar *name, void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  if (name != nullptr)
    ctx->devices_path_ = name;
}

int
PostureMerge::get_compress_cloud(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->compress_cloud_;
}

void
PostureMerge::set_compress_cloud(const int compress, void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  ctx->compress_cloud_ = compress;
}

int
PostureMerge::get_reload_calibration(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->reload_calibration_;
}

void
PostureMerge::set_reload_calibration(const int reload, void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  ctx->reload_calibration_ = reload;
}

int
PostureMerge::get_save_cloud(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->save_cloud_;
}

void
PostureMerge::set_save_cloud(const int save, void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  ctx->save_cloud_ = save;
}


int
PostureMerge::get_downsample_active(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->downsample_;
}

void
PostureMerge::set_downsample_active(const int active, void *user_data){
  PostureMerge *ctx = (PostureMerge *) user_data;

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
                                PostureMerge::set_downsampling_resolution,
                                PostureMerge::get_downsampling_resolution,
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

  if (ctx->merger_ != nullptr)
    ctx->merger_->setDownsampling(ctx->downsample_, ctx->downsample_resolution_);
}

double
PostureMerge::get_downsampling_resolution(void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  return ctx->downsample_resolution_;
}

void
PostureMerge::set_downsampling_resolution(const double resolution, void *user_data) {
  PostureMerge *ctx = (PostureMerge *) user_data;
  ctx->downsample_resolution_ = resolution;

  if (ctx->merger_ != nullptr)
    ctx->merger_->setDownsampling(ctx->downsample_, ctx->downsample_resolution_);
}

bool
PostureMerge::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE)
      || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

void
PostureMerge::free_sent_buffer(void* data)
{
  vector<unsigned char>* buffer = static_cast<vector<unsigned char>*>(data);
  buffer->clear();
}

void
PostureMerge::check_buffers()
{
  for (unsigned int i = 0; i < shmwriter_queue_.size();) {
    if (shmwriter_queue_[i]->size() == 0)
      shmwriter_queue_.erase(shmwriter_queue_.begin() + i);
    else
      i++;
  }
}
}  // namespace switcher
