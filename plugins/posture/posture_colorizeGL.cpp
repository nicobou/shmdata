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

#include "./posture_colorizeGL.hpp"

#include <iostream>
#include <regex>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureColorizeGL,
                                     "texturetomeshGL",
                                     "Project texture onto mesh using GPU",
                                     "video",
                                     "reader/writer",
                                     "Project texture onto mesh using GPU, based on calibration",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureColorizeGL::PostureColorizeGL(QuiddityConfiguration&&)
    : shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path) { return connect(path); },
                                  [this](const std::string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const std::string caps) { return can_sink_caps(caps); },
                                  std::numeric_limits<unsigned int>::max());

  pmanage<MPtr(&PContainer::make_string)>(
      "calibration_path",
      [this](const std::string& val) {
        calibration_path_ = val;
        if (calibration_reader_) {
          calibration_reader_->loadCalibration(calibration_path_);
          colorize_->setCalibration(calibration_reader_->getCalibrationParams());
        }
        return true;
      },
      [this]() { return calibration_path_; },
      "Calibration path",
      "Path to the calibration file",
      calibration_path_);

  pmanage<MPtr(&PContainer::make_bool)>("compress_mesh",
                                        [this](const bool& val) {
                                          compress_mesh_ = val;
                                          return true;
                                        },
                                        [this]() { return compress_mesh_; },
                                        "Compress the output mesh",
                                        "Compress the output mesh",
                                        compress_mesh_);
}

PostureColorizeGL::~PostureColorizeGL() { stop(); }

bool PostureColorizeGL::start() {
  if (is_started()) return false;

  calibration_reader_ = unique_ptr<CalibrationReader>(new CalibrationReader(calibration_path_));
  colorize_ = make_shared<ColorizeGL>();

  colorize_->setCalibration(calibration_reader_->getCalibrationParams());
  colorize_->setCompressMesh(compress_mesh_);

  return true;
}

bool PostureColorizeGL::stop() {
  lock_guard<mutex> lock(mutex_);

  if (colorize_ != nullptr) colorize_.reset();

  mesh_writer_.reset();
  tex_writer_.reset();

  has_input_mesh_ = false;
  mesh_index_ = -1;

  return true;
}

bool PostureColorizeGL::connect(std::string shmdata_socket_path) {
  unique_lock<mutex> connectLock(connect_mutex_);

  int id = source_id_;
  source_id_++;
  int shmid = shmdata_reader_id_;
  shmdata_reader_id_++;

  auto reader = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, int size) {
        if (!colorize_) return;

        if (!mutex_.try_lock()) return;

        auto typeIt = shmdata_reader_caps_.find(shmid);
        if (typeIt == shmdata_reader_caps_.end()) {
          mutex_.unlock();
          return;
        }
        auto type = typeIt->second;
        mutex_.unlock();

        unsigned int width, height, channels;
        // Update the input mesh. This calls the update of colorize_
        if (type == string(POLYGONMESH_TYPE_BASE) && size != 0) {
          if (!worker_.is_ready()) return;

          has_input_mesh_ = true;
          mesh_index_ = id;
          vector<unsigned char> mesh =
              vector<unsigned char>(reinterpret_cast<unsigned char*>(data),
                                    reinterpret_cast<unsigned char*>(data) + size);

          imageMutex_.lock();
          if (images_.size() == 0) {
            imageMutex_.unlock();
            return;
          }

          worker_.set_task([=]() {
            colorize_->setInput(std::move(mesh), images_, dims_);
            imageMutex_.unlock();

            auto texturedMesh = vector<unsigned char>();
            colorize_->getTexturedMesh(texturedMesh);

            unsigned int width, height;
            vector<unsigned char> texture = colorize_->getTexture(width, height);

            if (texturedMesh.size() == 0 || texture.size() == 0) return;

            // Write the mesh
            if (!mesh_writer_ ||
                texturedMesh.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
              auto data_type = string(POLYGONMESH_TYPE_BASE);
              mesh_writer_.reset();
              mesh_writer_ =
                  std::make_unique<ShmdataWriter>(this,
                                                  make_file_name("mesh"),
                                                  std::max(texturedMesh.size() * 2, (size_t)1024),
                                                  data_type);
            }

            mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
                const_cast<unsigned char*>(texturedMesh.data()), texturedMesh.size());
            mesh_writer_->bytes_written(texturedMesh.size());

            // Write the texture
            if (!tex_writer_ || width != prev_width_ || height != prev_height_) {
              auto data_type = "video/x-raw,format=(string)BGR,width=(int)" + to_string(width) +
                               ",height=(int)" + to_string(height) + ",framerate=30/1";
              tex_writer_.reset();
              tex_writer_ = std::make_unique<ShmdataWriter>(
                  this, make_file_name("texture"), texture.size(), data_type);
              prev_width_ = width;
              prev_height_ = height;
            }

            tex_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
                const_cast<unsigned char*>(texture.data()), texture.size());
            tex_writer_->bytes_written(texture.size());

          });
        }
        // Update the input textures
        else if (check_image_caps(type, width, height, channels)) {
          lock_guard<mutex> lock(imageMutex_);
          if (mesh_index_ != -1) {
            if (shm_index_.find(id) == shm_index_.end()) {
              // The following test is to handle the different input types (mesh
              // and image)
              if (id < mesh_index_)
                shm_index_[id] = id;
              else
                shm_index_[id] = id - 1;

              if (images_.size() < shm_index_[id] + 1) {
                images_.resize(shm_index_[id] + 1);
                dims_.resize(shm_index_[id] + 1);
              }
            }
            int index = shm_index_[id];

            images_[index] =
                vector<unsigned char>((unsigned char*)data, (unsigned char*)data + size);
            dims_[index] = vector<unsigned int>({width, height, channels});
          }
        }

      },
      [=](string caps) {
        unique_lock<mutex> lock(mutex_);
        shmdata_reader_caps_[shmid] = caps;
      });

  shmdata_readers_[shmdata_socket_path] = std::move(reader);
  return true;
}

bool PostureColorizeGL::disconnect(std::string /*unused*/) { return true; }

bool PostureColorizeGL::disconnect_all() { return true; }

bool PostureColorizeGL::can_sink_caps(std::string caps) {
  if (has_input_mesh_ && caps == POLYGONMESH_TYPE_BASE)
    return false;
  else if (caps == POLYGONMESH_TYPE_BASE)
    return true;

  unsigned int w, h, c;
  if (check_image_caps(caps, w, h, c)) return true;

  return false;
}

// Small function to work around a bug in GCC's libstdc++
void removeExtraParenthesis(string& str) {
  if (str.find(")") == 0) str = str.substr(1);
}

bool PostureColorizeGL::check_image_caps(string caps,
                                         unsigned int& width,
                                         unsigned int& height,
                                         unsigned int& channels) {
  regex regHap, regWidth, regHeight;
  regex regVideo, regFormat;
  try {
    regVideo = regex("(.*video/x-raw)(.*)", regex_constants::extended);
    regFormat = regex("(.*format=\\(string\\))(.*)", regex_constants::extended);
    regWidth = regex("(.*width=\\(int\\))(.*)", regex_constants::extended);
    regHeight = regex("(.*height=\\(int\\))(.*)", regex_constants::extended);
  } catch (const regex_error& e) {
    cout << "PostureColorizeGL::" << __FUNCTION__ << " - Regex error code: " << e.code() << endl;
    return false;
  }

  smatch match;
  string substr, format;

  if (regex_match(caps, regVideo)) {
    if (regex_match(caps, match, regFormat)) {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      removeExtraParenthesis(substr);
      substr = substr.substr(0, substr.find(","));

      if ("RGB" == substr) {
        channels = 3;
      } else if ("BGR" == substr) {
        channels = 3;
      } else {
        return false;
      }
    }

    if (regex_match(caps, match, regWidth)) {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      removeExtraParenthesis(substr);
      substr = substr.substr(0, substr.find(","));
      width = stoi(substr);
    } else {
      return false;
    }

    if (regex_match(caps, match, regHeight)) {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      removeExtraParenthesis(substr);
      substr = substr.substr(0, substr.find(","));
      height = stoi(substr);
    } else {
      return false;
    }

    return true;
  } else {
    return false;
  }
}

}  // namespace switcher
