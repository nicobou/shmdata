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

#include "./posture_meshserialization.hpp"

#include <iostream>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureMeshSerialization,
                                     "meshserialization",
                                     "Mesh Serialization",
                                     "video",
                                     "reader/writer",
                                     "Serialize / deserialize meshes",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureMeshSerialization::PostureMeshSerialization(QuiddityConfiguration&&)
    : shmcntr_(static_cast<Quiddity*>(this)) {
  serializer_ = make_shared<MeshSerializer>();
  // Default serializer parameters
  serializer_->setCompressionMethod(
      compression_method_, compression_level_, compression_precision_);
  shmcntr_.install_connect_method([this](const std::string path) { return connect(path); },
                                  [this](const std::string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const std::string caps) { return can_sink_caps(caps); },
                                  1);

  pmanage<MPtr(&PContainer::make_bool)>("compress",
                                        [this](const bool& val) {
                                          compress_ = val;
                                          if (val)
                                            compression_method_ = CTM_METHOD_MG1;
                                          else
                                            compression_method_ = CTM_METHOD_RAW;
                                          return true;
                                        },
                                        [this]() { return compress_; },
                                        "Compress",
                                        "Compress serialized mesh",
                                        compress_);

  pmanage<MPtr(&PContainer::make_double)>("compression precision",
                                          [this](const double& val) {
                                            compression_precision_ = val;
                                            return true;
                                          },
                                          [this]() { return compression_precision_; },
                                          "Grid step for compression",
                                          "Grid step for compression in mm",
                                          compression_precision_,
                                          0.001,
                                          0.1);

  pmanage<MPtr(&PContainer::make_int)>("compression level",
                                       [this](const int& val) {
                                         compression_level_ = val;
                                         return true;
                                       },
                                       [this]() { return compression_level_; },
                                       "Compression level",
                                       "Compression level: higher is more compressed",
                                       compression_level_,
                                       0,
                                       9);
}

bool PostureMeshSerialization::connect(std::string shmdata_socket_path) {
  mesh_reader_ = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        unique_lock<mutex> lock(mutex_);

        if (serializer_ == nullptr || (mesh_reader_caps_ != string(POLYGONMESH_TYPE_BASE))) return;

        // Update compression parameters
        serializer_->setCompressionMethod(
            compression_method_, compression_level_, compression_precision_);

        unsigned long long timestamp;
        auto mesh = serializer_->deserializeAsTextured(
            vector<uint8_t>((uint8_t*)data, (uint8_t*)data + size), timestamp);
        auto serializedMesh = serializer_->serialize(mesh, timestamp);

        if (serializedMesh.size() != 0) {
          if (!mesh_writer_) {
            auto data_type = string(POLYGONMESH_TYPE_BASE);
            mesh_writer_.reset();
            mesh_writer_ =
                std::make_unique<ShmdataWriter>(this,
                                                make_file_name("mesh"),
                                                std::max(serializedMesh.size(), (size_t)1024),
                                                data_type);
          }

          mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
              const_cast<unsigned char*>(serializedMesh.data()), serializedMesh.size());
          mesh_writer_->bytes_written(serializedMesh.size());
        }
      },
      [=](string caps) {
        unique_lock<mutex> lock(mutex_);
        mesh_reader_caps_ = caps;
      });

  return true;
}

bool PostureMeshSerialization::disconnect(std::string) {
  unique_lock<mutex> lock(mutex_);
  mesh_reader_.reset();
  return true;
}

bool PostureMeshSerialization::disconnect_all() {
  disconnect("");
  return true;
}

bool PostureMeshSerialization::can_sink_caps(std::string caps) {
  return (caps == POLYGONMESH_TYPE_BASE);
}

}  // namespace switcher
