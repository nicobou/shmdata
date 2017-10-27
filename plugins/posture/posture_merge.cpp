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
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureMerge,
                                     "pclmergesink",
                                     "Point Clouds Merge",
                                     "video",
                                     "writer/reader",
                                     "Merges point clouds captured with 3D cameras",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureMerge::~PostureMerge() { stop(); }

bool PostureMerge::start() {
  calibration_reader_ = unique_ptr<CalibrationReader>(new CalibrationReader(calibration_path_));
  merger_ = make_shared<PointCloudMerger>();
  merger_->setCloudNbr(source_id_);
  merger_->setCalibration(calibration_reader_->getCalibrationParams());
  merger_->setCompression(compress_cloud_);
  merger_->setSaveCloud(save_cloud_);
  merger_->start();
  return true;
}

bool PostureMerge::stop() {
  lock_guard<mutex> lock(mutex_);

  if (merger_ != nullptr) {
    merger_->stop();
    merger_.reset();
  }

  if (cloud_writer_ != nullptr) {
    cloud_writer_.reset();
  }

  return true;
}

PostureMerge::PostureMerge(QuiddityConfiguration&&) : shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path) { return connect(path); },
                                  [this](const std::string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const std::string caps) { return can_sink_caps(caps); },
                                  8);

  pmanage<MPtr(&PContainer::make_string)>(
      "calibration_path",
      [this](const std::string& val) {
        calibration_path_ = val;
        if (calibration_reader_) {
          calibration_reader_->loadCalibration(calibration_path_);
          merger_->setCalibration(calibration_reader_->getCalibrationParams());
        }
        return true;
      },
      [this]() { return calibration_path_; },
      "Calibration path",
      "Path to the calibration file",
      calibration_path_);

  pmanage<MPtr(&PContainer::make_bool)>("compress_cloud",
                                        [this](const bool& val) {
                                          compress_cloud_ = val;
                                          return true;
                                        },
                                        [this]() { return compress_cloud_; },
                                        "Compression",
                                        "Compress the cloud if true",
                                        compress_cloud_);

  pmanage<MPtr(&PContainer::make_bool)>("reload_calibration",
                                        [this](const bool& val) {
                                          reload_calibration_ = val;
                                          return true;
                                        },
                                        [this]() { return reload_calibration_; },
                                        "Reload calibration",
                                        "Reload calibration at each frame",
                                        reload_calibration_);

  pmanage<MPtr(&PContainer::make_bool)>("save_cloud",
                                        [this](const bool& val) {
                                          save_cloud_ = val;
                                          return true;
                                        },
                                        [this]() { return save_cloud_; },
                                        "Save current cloud",
                                        "Save the current cloud if true",
                                        save_cloud_);

  pmanage<MPtr(&PContainer::make_bool)>(
      "downsample",
      [this](const bool& active) {
        if (downsample_ != active && active == true) {
          downsample_ = active;
          downsample_resolution_id_ = pmanage<MPtr(&PContainer::make_double)>(
              "downsample_resolution",
              [this](const double& val) {
                downsample_resolution_ = val;
                if (merger_ != nullptr)
                  merger_->setDownsampling(downsample_, downsample_resolution_);
                return true;
              },
              [this]() { return downsample_resolution_; },
              "Resampling resolution",
              "Resampling resolution",
              downsample_resolution_,
              0.01,
              1.0);
        } else if (downsample_ != active && active == false) {
          downsample_ = false;
          pmanage<MPtr(&PContainer::remove)>(downsample_resolution_id_);
        }
        if (merger_ != nullptr) merger_->setDownsampling(downsample_, downsample_resolution_);
        return true;
      },
      [this]() { return downsample_; },
      "Downsample",
      "Activate the cloud downsampling",
      downsample_);
}

bool PostureMerge::connect(std::string shmdata_socket_path) {
  unique_lock<mutex> connectLock(connect_mutex_);

  int index = source_id_;
  source_id_ += 1;
  int shmreader_id = shmreader_id_;
  shmreader_id_++;

  auto reader = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        // If another thread is trying to get the merged cloud, stock him and
        // don't bother
        if (!mutex_.try_lock()) {
          unique_lock<mutex> lock(stock_mutex_);
          stock_[index] = vector<char>((char*)data, (char*)data + size);
          return;
        }

        // Test if we already received the type
        auto typeIt = cloud_readers_caps_.find(shmreader_id);
        if (typeIt == cloud_readers_caps_.end()) {
          mutex_.unlock();
          return;
        }
        string type = typeIt->second;

        if (merger_ == nullptr ||
            (type != string(POINTCLOUD_TYPE_BASE) && type != string(POINTCLOUD_TYPE_COMPRESSED))) {
          mutex_.unlock();
          return;
        }

        if (reload_calibration_) {
          calibration_reader_->loadCalibration(calibration_path_);
          merger_->setCalibration(calibration_reader_->getCalibrationParams());
        }

        // Setting input clouds is thread safe, so lets do it
        {
          unique_lock<mutex> lock(stock_mutex_);
          for (auto it = stock_.begin(); it != stock_.end(); ++it) {
            merger_->setInputCloud(it->first, it->second, type != string(POINTCLOUD_TYPE_BASE));
          }
          stock_.clear();
        }

        merger_->setInputCloud(index,
                               vector<char>((char*)data, (char*)data + size),
                               type != string(POINTCLOUD_TYPE_BASE));
        auto cloud = vector<char>();
        merger_->getCloud(cloud);

        if (cloud_writer_.get() == nullptr ||
            cloud.size() > cloud_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
          auto data_type =
              compress_cloud_ ? string(POINTCLOUD_TYPE_COMPRESSED) : string(POINTCLOUD_TYPE_BASE);
          cloud_writer_.reset();
          cloud_writer_ = std::make_unique<ShmdataWriter>(
              this, make_file_name("cloud"), std::max(cloud.size() * 2, (size_t)1024), data_type);
        }

        cloud_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(const_cast<char*>(cloud.data()),
                                                                   cloud.size());
        cloud_writer_->bytes_written(cloud.size());

        mutex_.unlock();
      },
      [=](string caps) {
        unique_lock<mutex> lock(mutex_);
        cloud_readers_caps_[shmreader_id] = caps;
      });

  cloud_readers_[shmdata_socket_path] = std::move(reader);
  return true;
}

bool PostureMerge::disconnect(std::string shmName) {
  unique_lock<mutex> lock(mutex_);
  try {
    cloud_readers_.erase(shmName);
  } catch (...) {
    warning("An exception has been caught while trying to disconnect from shmdata %", shmName);
  }
  return true;
}

bool PostureMerge::disconnect_all() {
  source_id_ = 0;
  return true;
}

bool PostureMerge::can_sink_caps(std::string caps) {
  return (caps == POINTCLOUD_TYPE_BASE) || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

}  // namespace switcher
