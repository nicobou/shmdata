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

#include "./posture_solidify.hpp"

#include <iostream>
#include <thread>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSolidify,
                                     "Point Clouds to Mesh",
                                     "video",
                                     "Convert a point cloud to a mesh",
                                     "LGPL",
                                     "pcltomeshsink", "Emmanuel Durand");

PostureSolidify::PostureSolidify():
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

PostureSolidify::~PostureSolidify() {
  stop();
}

bool
PostureSolidify::start() {
  solidify_ = make_shared<Solidify>();
  solidify_->setGridResolution(marching_cubes_resolution_);
  solidify_->setSaveMesh(save_mesh_);

  return true;
}

bool
PostureSolidify::stop() {
  lock_guard<mutex> lock(mutex_);
  solidify_.reset();

  if (mesh_writer_ != nullptr)
  {
    unregister_shmdata(mesh_writer_->get_path());
    mesh_writer_.reset();
  }

  return true;
}

bool
PostureSolidify::init() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&PostureSolidify::connect, this, std::placeholders::_1), nullptr,     // FIXME implement this (disconnect with the shmdata as unique argument)
                         std::bind(&PostureSolidify::disconnect_all, this),
                         std::bind(&PostureSolidify::can_sink_caps, this, std::placeholders::_1),
                         1);

  save_mesh_prop_ = custom_props_->make_boolean_property("save_mesh",
                                           "Save the current mesh if true",
                                           save_mesh_,
                                           (GParamFlags) G_PARAM_READWRITE,
                                           PostureSolidify::set_save_mesh,
                                           PostureSolidify::get_save_mesh,
                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            save_mesh_prop_, "save_mesh",
                            "Save the current mesh if true");

  marching_cubes_resolution_prop_ = custom_props_->make_int_property("marching_cubes_resolution",
                                      "Resolution of the marching cubes reconstruction",
                                      8,
                                      256,
                                      marching_cubes_resolution_,
                                      (GParamFlags)
                                      G_PARAM_READWRITE,
                                      PostureSolidify::set_marching_cubes_resolution,
                                      PostureSolidify::get_marching_cubes_resolution,
                                      this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            marching_cubes_resolution_prop_, "marching_cubes_resolution",
                            "Resolution of the marching cubes reconstruction");

  return true;
}

bool
PostureSolidify::connect(std::string shmdata_socket_path) {
  ShmdataAnyReader::ptr reader_ = make_shared<ShmdataAnyReader>();
  reader_->set_path(shmdata_socket_path);

  // This is the callback for when new clouds are received
  reader_->set_callback([=] (void *data,
                             int size,
                             unsigned long long /*unused*/,
                             const char *type,
                             void * /*unused */ )
  {
    if (solidify_ == nullptr || (string(type) != string(POINTCLOUD_TYPE_BASE) && string(type) != string(POINTCLOUD_TYPE_COMPRESSED)))
      return;

    if (!mutex_.try_lock())
      return;

    // Setting input clouds is thread safe, so lets do it
    solidify_->setInputCloud(vector<char>((char*)data, (char*) data + size),
                             string(type) != string(POINTCLOUD_TYPE_BASE));

    auto computeThread = thread([=] () {
      // Get the result mesh, and send it through shmdata
      vector<unsigned char> mesh = solidify_->getMesh();
      if (mesh_writer_ == nullptr)
      {
        mesh_writer_ = make_shared<ShmdataAnyWriter>();
        mesh_writer_->set_path(make_file_name("mesh"));
        mesh_writer_->set_data_type(POLYGONMESH_TYPE_BASE);
        register_shmdata(mesh_writer_);
        mesh_writer_->start();
      }

      check_buffers();
      shmwriter_queue_.push_back(make_shared<vector<unsigned char>>(reinterpret_cast<const unsigned char*>(mesh.data()),
                                                                    reinterpret_cast<const unsigned char*>(mesh.data()) + mesh.size()));
      mesh_writer_->push_data_auto_clock((void *) shmwriter_queue_[shmwriter_queue_.size() - 1]->data(),
                                          mesh.size(),
                                          PostureSolidify::free_sent_buffer,
                                          (void*)(shmwriter_queue_[shmwriter_queue_.size() - 1].get()));
      mutex_.unlock();
    });

    computeThread.detach();
  },
  nullptr);

  reader_->start();
  register_shmdata(reader_);
  return true;
}

bool
PostureSolidify::disconnect_all() {
  if (solidify_ == nullptr)
    clear_shmdatas();
  return true;
}

bool
PostureSolidify::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE)
      || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

int
PostureSolidify::get_marching_cubes_resolution(void *user_data) {
  PostureSolidify *ctx = (PostureSolidify *) user_data;
  return ctx->marching_cubes_resolution_;
}

void
PostureSolidify::set_marching_cubes_resolution(const int res, void *user_data) {
  PostureSolidify *ctx = (PostureSolidify *) user_data;
  ctx->marching_cubes_resolution_ = res;

  if (ctx->solidify_ != nullptr)
    ctx->solidify_->setGridResolution(res);
}

int
PostureSolidify::get_save_mesh(void *user_data) {
  PostureSolidify *ctx = (PostureSolidify *) user_data;
  return ctx->save_mesh_;
}

void
PostureSolidify::set_save_mesh(const int save, void *user_data) {
  PostureSolidify *ctx = (PostureSolidify *) user_data;
  ctx->save_mesh_ = save;

  if (ctx->solidify_ != nullptr)
    ctx->solidify_->setSaveMesh(save);
}

void
PostureSolidify::free_sent_buffer(void* data)
{
  vector<unsigned char>* buffer = static_cast<vector<unsigned char>*>(data);
  buffer->clear();
}

void
PostureSolidify::check_buffers()
{
  for (unsigned int i = 0; i < shmwriter_queue_.size();) {
    if (shmwriter_queue_[i]->size() == 0)
      shmwriter_queue_.erase(shmwriter_queue_.begin() + i);
    else
      i++;
  }
}

}  // namespace switcher
