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
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureMeshMerge,
                                     "Mesh Merge",
                                     "video",
                                     "Merges meshes captured with 3D cameras",
                                     "LGPL",
                                     "meshmergesink", "Emmanuel Durand");

PostureMeshMerge::PostureMeshMerge(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
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

  clear_shmdatas();
  mesh_writer_.reset();

  return true;
}

bool
PostureMeshMerge::init() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&PostureMeshMerge::connect, this, std::placeholders::_1),
                         std::bind(&PostureMeshMerge::disconnect, this, std::placeholders::_1),
                         std::bind(&PostureMeshMerge::disconnect_all, this),
                         std::bind(&PostureMeshMerge::can_sink_caps, this, std::placeholders::_1),
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
  int index = source_id_;
  source_id_ += 1;

  ShmdataAnyReader::ptr reader = make_shared<ShmdataAnyReader>();
  reader->set_path(shmdata_socket_path);

  // This is the callback for when new clouds are received
  reader->set_callback([=] (void *data,
                             int size,
                             unsigned long long /*unused*/,
                             const char *type,
                             void * /*unused */ )
  {
    if (merger_ == nullptr || (string(type) != string(POLYGONMESH_TYPE_BASE)))
      return;

    // Setting input mesh is thread safe, so lets do it
    merger_->setInputMesh(index, vector<unsigned char>((unsigned char*)data, (unsigned char*)data + size));

    if (!worker_.is_ready() || !updateMutex_.try_lock())
      return;

    worker_.set_task([&]() {
      if (reload_calibration_)
          merger_->reloadCalibration();

      if (mesh_writer_ == nullptr) {
        mesh_writer_.reset(new ShmdataAnyWriter);
        mesh_writer_->set_path(make_file_name("mesh"));
        register_shmdata(mesh_writer_);
        mesh_writer_->set_data_type(string(POLYGONMESH_TYPE_BASE));
        mesh_writer_->start();
      }

      check_buffers();
      vector<unsigned char> mesh = merger_->getMesh();
      shmwriter_queue_.push_back(make_shared<vector<unsigned char>>(mesh));
      mesh_writer_->push_data_auto_clock((void *) shmwriter_queue_[shmwriter_queue_.size() - 1]->data(),
                                          mesh.size(),
                                          PostureMeshMerge::free_sent_buffer,
                                          (void*)(shmwriter_queue_[shmwriter_queue_.size() - 1].get()));

      updateMutex_.unlock();
    });

    worker_.do_task();
  },
  nullptr);

  reader->start();
  register_shmdata(reader);
  return true;
}

bool
PostureMeshMerge::disconnect(std::string shmName) {
  std::lock_guard<mutex> lock(mutex_);
  unregister_shmdata(shmName);
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

void
PostureMeshMerge::free_sent_buffer(void* data)
{
  vector<unsigned char>* buffer = static_cast<vector<unsigned char>*>(data);
  buffer->clear();
}

void
PostureMeshMerge::check_buffers()
{
  for (unsigned int i = 0; i < shmwriter_queue_.size();) {
    if (shmwriter_queue_[i]->size() == 0)
      shmwriter_queue_.erase(shmwriter_queue_.begin() + i);
    else
      i++;
  }
}
}  // namespace switcher
