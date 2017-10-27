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

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSolidify,
                                     "pcltomeshsink",
                                     "Point Clouds to Mesh",
                                     "video",
                                     "writer/reader",
                                     "Convert a point cloud to a mesh",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureSolidify::PostureSolidify(QuiddityConfiguration&&) : shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path) { return connect(path); },
                                  [this](const std::string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const std::string caps) { return can_sink_caps(caps); },
                                  1);

  pmanage<MPtr(&PContainer::make_bool)>("save_mesh",
                                        [this](const bool& val) {
                                          save_mesh_ = val;
                                          if (solidify_ != nullptr) solidify_->setSaveMesh(val);
                                          return true;
                                        },
                                        [this]() { return save_mesh_; },
                                        "Save mesh",
                                        "Save the current mesh if true",
                                        save_mesh_);

  pmanage<MPtr(&PContainer::make_int)>("marching_cubes_resolution",
                                       [this](const int& val) {
                                         marching_cubes_resolution_ = val;
                                         if (solidify_ != nullptr)
                                           solidify_->setGridResolution(val);
                                         return true;
                                       },
                                       [this]() { return marching_cubes_resolution_; },
                                       "Marching cubes resolution",
                                       "Resolution of the marching cubes reconstruction",
                                       marching_cubes_resolution_,
                                       8,
                                       256);
}

PostureSolidify::~PostureSolidify() { stop(); }

bool PostureSolidify::start() {
  solidify_ = make_shared<Solidify>();
  solidify_->setGridResolution(marching_cubes_resolution_);
  solidify_->setSaveMesh(save_mesh_);

  return true;
}

bool PostureSolidify::stop() {
  lock_guard<mutex> lock(mutex_);
  solidify_.reset();

  return true;
}

bool PostureSolidify::connect(std::string shmdata_socket_path) {
  pcl_reader_ = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        if (!worker_.is_ready() || !mutex_.try_lock()) return;

        if (solidify_ == nullptr || (pcl_reader_caps_ != string(POINTCLOUD_TYPE_BASE) &&
                                     pcl_reader_caps_ != string(POINTCLOUD_TYPE_COMPRESSED))) {
          mutex_.unlock();
          return;
        }

        // Setting input clouds is thread safe, so lets do it
        solidify_->setInputCloud(vector<char>((char*)data, (char*)data + size),
                                 pcl_reader_caps_ != string(POINTCLOUD_TYPE_BASE));

        worker_.set_task([=]() {
          // Get the result mesh, and send it through shmdata
          // FIXME this is not required since shmdata is resizing
          auto mesh = vector<unsigned char>();
          solidify_->getMesh(mesh);
          if (mesh_writer_ == nullptr ||
              mesh.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
            auto data_type = string(POLYGONMESH_TYPE_BASE);
            mesh_writer_.reset();
            mesh_writer_ = std::make_unique<ShmdataWriter>(
                this, make_file_name("mesh"), std::max(mesh.size() * 2, (size_t)1024), data_type);
          }

          mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
              const_cast<unsigned char*>(mesh.data()), mesh.size());
          mesh_writer_->bytes_written(mesh.size());

          mutex_.unlock();
        });
      },
      [=](string caps) {
        unique_lock<mutex> lock(mutex_);
        pcl_reader_caps_ = caps;
      });

  return true;
}

bool PostureSolidify::disconnect(std::string /*unused*/) { return disconnect_all(); }

bool PostureSolidify::disconnect_all() {
  std::lock_guard<mutex> lock(mutex_);
  stop();
  return true;
}

bool PostureSolidify::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE) || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

}  // namespace switcher
