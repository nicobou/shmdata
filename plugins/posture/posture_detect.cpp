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

#include "./posture_detect.hpp"

#include <iostream>
#include <thread>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureDetect,
                                     "pcldetectsink",
                                     "Point Cloud Detect",
                                     "video",
                                     "reader/writer",
                                     "Detect shapes and objects in point clouds",
                                     "LGPL",
                                     "Emmanuel Durand");


PostureDetect::~PostureDetect() { stop(); }

bool PostureDetect::start() {
  detect_ = make_shared<Detect>();

  return true;
}

bool PostureDetect::stop() {
  lock_guard<mutex> lock(mutex_);

  cloud_writer_.reset();
  mesh_writer_.reset();

  detect_.reset();

  return true;
}

PostureDetect::PostureDetect(QuiddityConfiguration&&) : shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);
  shmcntr_.install_connect_method([this](const string path) { return connect(path); },
                                  [this](const string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const string caps) { return can_sink_caps(caps); },
                                  1);
}

bool PostureDetect::connect(std::string shmdata_socket_path) {
  reader_ = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        // If another thread is trying to get the merged cloud, don't bother
        if (!mutex_.try_lock()) return;

        if (detect_ == nullptr || (reader_caps_ != string(POINTCLOUD_TYPE_BASE) &&
                                   reader_caps_ != string(POINTCLOUD_TYPE_COMPRESSED))) {
          mutex_.unlock();
          return;
        }

        // Setting input clouds is thread safe, so lets do it
        detect_->setInputCloud(vector<char>((char*)data, (char*)data + size),
                               reader_caps_ != string(POINTCLOUD_TYPE_BASE));

        thread computeThread = thread([&]() {
          if (detect_->detect()) {
            auto cloud = vector<char>();
            detect_->getProcessedCloud(cloud);
            auto poly = vector<unsigned char>();
            detect_->getConvexHull(poly, 0);

            auto data_type =
                compress_cloud_ ? string(POINTCLOUD_TYPE_COMPRESSED) : string(POINTCLOUD_TYPE_BASE);
            if (!cloud_writer_ ||
                cloud.size() > cloud_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
              cloud_writer_.reset();
              cloud_writer_ =
                  std::make_unique<ShmdataWriter>(this,
                                                  make_file_name("cloud"),
                                                  std::max(cloud.size() * 2, (size_t)1024),
                                                  data_type);
            }

            data_type = string(POLYGONMESH_TYPE_BASE);
            if (!mesh_writer_ ||
                poly.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
              // FIXME manual resize is not required since already done in shmdata
              mesh_writer_.reset();
              mesh_writer_ = std::make_unique<ShmdataWriter>(
                  this, make_file_name("mesh"), std::max(poly.size() * 2, (size_t)1024), data_type);
            }

            cloud_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
                const_cast<char*>(cloud.data()), cloud.size());
            cloud_writer_->bytes_written(cloud.size());

            mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
                const_cast<unsigned char*>(poly.data()), poly.size());
            mesh_writer_->bytes_written(cloud.size());
          }

          mutex_.unlock();
        });

        computeThread.detach();
      },
      [=](const string caps) {
        unique_lock<mutex> lock(mutex_);
        reader_caps_ = caps;
      });

  return true;
}

bool PostureDetect::disconnect(std::string) {
  std::lock_guard<mutex> lock(mutex_);
  return true;
}

bool PostureDetect::disconnect_all() { return true; }

bool PostureDetect::can_sink_caps(string caps) {
  return (caps == POINTCLOUD_TYPE_BASE) || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

}  // namespace switcher
