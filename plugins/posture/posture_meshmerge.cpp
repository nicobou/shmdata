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

#include "./posture_meshmerge.hpp"

#include <iostream>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PostureMeshMerge,
    "meshmerge",
    "Mesh Merge",
    "video",
    "reader/writer",
    "Merges meshes captured with 3D cameras",
    "LGPL",
    "Emmanuel Durand");

PostureMeshMerge::PostureMeshMerge(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()),
    shmcntr_(static_cast<Quiddity*>(this)) {
}

PostureMeshMerge::~PostureMeshMerge() {
  stop();
}

bool
PostureMeshMerge::start() {
  merger_ = make_shared<MeshMerger>("", source_id_);

  merger_->setCalibrationPath(calibration_path_);
  merger_->setApplyCalibration(apply_calibration_);
  merger_->setDevicesPath(devices_path_);

  merger_->start();

  return true;
}

bool
PostureMeshMerge::stop() {
  lock_guard<mutex> lock(mutex_);
  lock_guard<mutex> lockUpdate(updateMutex_);

  if (merger_ != nullptr) {
    merger_->stop();
    merger_.reset();
  }

  return true;
}

bool
PostureMeshMerge::init() {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path){return connect(path);},
                                  [this](const std::string path){return disconnect(path);},
                                  [this](){return disconnect_all();},
                                  [this](const std::string caps){return can_sink_caps(caps);},
                                  8);

  calibration_path_prop_ =
      custom_props_->make_string_property("calibration_path",
                                          "Path to the calibration file",
                                          calibration_path_.c_str(),
                                          (GParamFlags) G_PARAM_READWRITE,
                                          PostureMeshMerge::set_calibration_path,
                                          PostureMeshMerge::get_calibration_path,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            calibration_path_prop_, "calibration_path",
                            "Path to the calibration file");

  devices_path_prop_ = custom_props_->make_string_property("devices_path",
                                                           "Path to the devices description file",
                                                           devices_path_.c_str
                                                           (), (GParamFlags)
                                                           G_PARAM_READWRITE,
                                                           PostureMeshMerge::set_devices_path,
                                                           PostureMeshMerge::get_devices_path,
                                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            devices_path_prop_, "devices",
                            "Path to the devices description file");

  reload_calibration_prop_ = custom_props_->make_boolean_property("reload_calibration",
                                "Reload calibration at each frame",
                                reload_calibration_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureMeshMerge::set_reload_calibration,
                                PostureMeshMerge::get_reload_calibration,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            reload_calibration_prop_, "reload_calibration",
                            "Reload calibration at each frame");

  apply_calibration_prop_ = custom_props_->make_boolean_property("apply_calibration",
                                "Apply loaded calibration to meshes",
                                apply_calibration_,
                                (GParamFlags) G_PARAM_READWRITE,
                                PostureMeshMerge::set_apply_calibration,
                                PostureMeshMerge::get_apply_calibration,
                                this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            apply_calibration_prop_, "apply_calibration",
                            "Apply loaded calibration to meshes");

  return true;
}

bool
PostureMeshMerge::connect(std::string shmdata_socket_path) {
  unique_lock<mutex> connectLock(connect_mutex_);

  int index = source_id_;
  source_id_ += 1;
  int shmreader_id = shmreader_id_;
  shmreader_id_++;

  auto reader = std2::make_unique<ShmdataFollower>(this,
                  shmdata_socket_path,
                  [=] (void *data, size_t size) {
    if (!mutex_.try_lock())
    {
      unique_lock<mutex> lock(stock_mutex_);
      stock_[index] = vector<unsigned char>((unsigned char*)data, (unsigned char*)data + size);
      return;
    }

    auto typeIt = mesh_readers_caps_.find(shmreader_id);
    if (typeIt == mesh_readers_caps_.end())
    {
      mutex_.unlock();
      return;
    }
    string type = typeIt->second;
    mutex_.unlock();

    if (merger_ == nullptr || (type != string(POLYGONMESH_TYPE_BASE)))
    {
      return;
    }

    // Setting input meshes is thread safe, so lets do it
    {
      unique_lock<mutex> lock(stock_mutex_);
      for (auto it = stock_.begin(); it != stock_.end(); ++it)
      {
        merger_->setInputMesh(it->first,
                              it->second);
      }
      stock_.clear();
    }


    merger_->setInputMesh(index, vector<unsigned char>((unsigned char*)data, (unsigned char*)data + size));

    if (!worker_.is_ready() || !updateMutex_.try_lock())
      return;

    worker_.set_task([&]() {
      if (reload_calibration_)
          merger_->reloadCalibration();

      auto mesh = vector<unsigned char>();
      merger_->getMesh(mesh);

      if (mesh_writer_ == nullptr || mesh.size() > mesh_writer_->writer(&shmdata::Writer::alloc_size)) {
        auto data_type = string(POLYGONMESH_TYPE_BASE);
        mesh_writer_.reset();
        mesh_writer_ = std2::make_unique<ShmdataWriter>(this,
                                                        make_file_name("mesh"),
                                                        std::max(mesh.size() * 2, (size_t)1024),
                                                        data_type);
      }

      mesh_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<unsigned char*>(mesh.data()), mesh.size());
      mesh_writer_->bytes_written(mesh.size());

      updateMutex_.unlock();
    });

    worker_.do_task();
  }, [=](string caps) {
    unique_lock<mutex> lock(mutex_);
    mesh_readers_caps_[shmreader_id] = caps;
  });

  mesh_readers_[shmdata_socket_path] = std::move(reader);
  return true;
}

bool
PostureMeshMerge::disconnect(std::string) {
  std::lock_guard<mutex> lock(mutex_);
  return true;
}

bool
PostureMeshMerge::disconnect_all() {
  source_id_ = 0;
  return true;
}

const gchar *
PostureMeshMerge::get_calibration_path(void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  return ctx->calibration_path_.c_str();
}

void
PostureMeshMerge::set_calibration_path(const gchar *name, void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  if (name != nullptr)
    ctx->calibration_path_ = name;
}

const gchar *
PostureMeshMerge::get_devices_path(void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  return ctx->devices_path_.c_str();
}

void
PostureMeshMerge::set_devices_path(const gchar *name, void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  if (name != nullptr)
    ctx->devices_path_ = name;
}

int
PostureMeshMerge::get_reload_calibration(void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  return ctx->reload_calibration_;
}

void
PostureMeshMerge::set_reload_calibration(const int reload, void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  ctx->reload_calibration_ = reload;
}

int
PostureMeshMerge::get_apply_calibration(void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  return ctx->apply_calibration_;
}

void
PostureMeshMerge::set_apply_calibration(const int apply, void *user_data) {
  PostureMeshMerge *ctx = (PostureMeshMerge *) user_data;
  ctx->apply_calibration_ = apply;
}

bool
PostureMeshMerge::can_sink_caps(std::string caps) {
  return (caps == POLYGONMESH_TYPE_BASE);
}

}  // namespace switcher
