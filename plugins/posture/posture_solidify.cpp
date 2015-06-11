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
#include "switcher/std2.hpp"

#include <iostream>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PostureSolidify,
    "pcltomeshsink",
    "Point Clouds to Mesh",
    "video",
    "writer/reader",
    "Convert a point cloud to a mesh",
    "LGPL",
    "Emmanuel Durand");

PostureSolidify::PostureSolidify(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()),
    shmcntr_(static_cast<Quiddity*>(this)) {
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

  return true;
}

bool
PostureSolidify::init() {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path){return connect(path);},
                                  [this](const std::string path){return disconnect(path);},
                                  [this](){return disconnect_all();},
                                  [this](const std::string caps){return can_sink_caps(caps);},
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
  pcl_reader_ = std2::make_unique<ShmdataFollower>(this,
                                                   shmdata_socket_path,
                                                   [=] (void *data, size_t size) {
    if (!worker_.is_ready() || !mutex_.try_lock())
      return;

    if (solidify_ == nullptr || (pcl_reader_caps_ != string(POINTCLOUD_TYPE_BASE) && pcl_reader_caps_ != string(POINTCLOUD_TYPE_COMPRESSED)))
    {
      mutex_.unlock();
      return;
    }

    // Setting input clouds is thread safe, so lets do it
    solidify_->setInputCloud(vector<char>((char*)data, (char*) data + size),
                             pcl_reader_caps_ != string(POINTCLOUD_TYPE_BASE));

    worker_.set_task([=] () {
      // Get the result mesh, and send it through shmdata
      auto mesh = vector<unsigned char>();
      solidify_->getMesh(mesh);
      if (mesh_writer_ == nullptr || mesh.size() > mesh_writer_->writer(&shmdata::Writer::alloc_size))
      {
        auto data_type = string(POLYGONMESH_TYPE_BASE);
        mesh_writer_.reset();
        mesh_writer_ = std2::make_unique<ShmdataWriter>(this,
                                                        make_file_name("mesh"),
                                                        std::max(mesh.size() * 2, (size_t)1024),
                                                        data_type);
      }

      mesh_writer_->writer(&shmdata::Writer::copy_to_shm, const_cast<unsigned char*>(mesh.data()), mesh.size());
      mesh_writer_->bytes_written(mesh.size());

      mutex_.unlock();
    });

    worker_.do_task();
  }, [=](string caps) {
    unique_lock<mutex> lock(mutex_);
    pcl_reader_caps_ = caps;
  });

  return true;
}

bool
PostureSolidify::disconnect(std::string /*unused*/) {
  return disconnect_all();
}

bool
PostureSolidify::disconnect_all() {
  std::lock_guard<mutex> lock(mutex_);
  stop();
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

}  // namespace switcher
