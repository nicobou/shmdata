/*
 * This file is part of posture.
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

#include "./posture_scan3d.hpp"

#include <functional>
#include <iostream>

#include <pcl/io/obj_io.h>
#include <boost/make_shared.hpp>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureSc3,
                                     "posturescansrc",
                                     "Scan 3D",
                                     "video",
                                     "writer",
                                     "Grabs meshes using zcameras",
                                     "LGPL",
                                     "Ludovic Schreiber");

PostureSc3::PostureSc3(QuiddityConfiguration&&) {
  merger_ = std::unique_ptr<PointCloudMerger>(new PointCloudMerger());
  sol_ = std::unique_ptr<Solidify>(new Solidify());
  sol_->setGridResolution(50);
  colorize_ = std::unique_ptr<Colorize>(new Colorize());
  intermediate_mesh_ = boost::make_shared<pcl::PolygonMesh>();

  init_startable(this);

  auto cam_id = pmanage<MPtr(&PContainer::make_int)>(
      "camera_number",
      [this](const int& val) {
        nbr_ = val;
        cameras_.resize(nbr_);
        merger_->setCloudNbr(nbr_);
        for (index_ = 0; index_ < nbr_; index_++) {
          std::shared_ptr<ZCamera> newCam = std::shared_ptr<ZCamera>(new ZCamera());
          cameras_[index_] = newCam;
          cameras_[index_]->setDeviceIndex(index_);
          cameras_[index_]->setCaptureMode(ZCamera::CaptureMode::QQVGA_30Hz);
          int index = index_;
          cameras_[index_]->setCallbackCloud(
              [=](void*, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) -> void {
                cb_frame_cloud(index, std::move(cloud));
              },
              nullptr);
        }
        return true;
      },
      [this]() { return nbr_; },
      "Number of camera",
      "Number of used cameras",
      nbr_,
      1,
      9);
  pmanage<MPtr(&PContainer::set<int>)>(cam_id, nbr_);

  pmanage<MPtr(&PContainer::make_string)>(
      "calibration_path",
      [this](const std::string& val) {
        calibration_path_ = val;
        if (!calibration_reader_)
          calibration_reader_ =
              unique_ptr<CalibrationReader>(new CalibrationReader(calibration_path_));
        merger_->setCalibration(calibration_reader_->getCalibrationParams());
        return true;
      },
      [this]() { return calibration_path_; },
      "Calibration path",
      "Path to the calibration file",
      calibration_path_);

  pmanage<MPtr(&PContainer::make_int)>("grid resolution",
                                       [this](const int& val) {
                                         grid_res_ = val;
                                         sol_->setGridResolution(grid_res_);
                                         return true;
                                       },
                                       [this]() { return grid_res_; },
                                       "Grid resolution",
                                       "Resolution for the mesh reconstruction",
                                       grid_res_,
                                       3,
                                       99);

  pmanage<MPtr(&PContainer::make_bool)>("reload_calibration",
                                        [this](const bool& val) {
                                          reload_calibration_ = val;
                                          return true;
                                        },
                                        [this]() { return reload_calibration_; },
                                        "Reload calibration",
                                        "Reload calibration at each frame",
                                        reload_calibration_);

  pmanage<MPtr(&PContainer::make_bool)>(
      "colorize",
      [this](const bool& val) {
        colorize_or_not_ = val;
        for (index_ = 0; index_ < nbr_; index_++) {
          if (colorize_or_not_) {
            cameras_[index_]->setCallbackRgb(
                [=](void*, std::vector<unsigned char>& image, int width, int heigth) -> void {
                  cb_frame_rgb(image, width, heigth);
                },
                nullptr);
          }
        }
        return true;
      },
      [this]() { return colorize_or_not_; },
      "Colorize",
      "Check for a colorize mesh",
      colorize_or_not_);
}

PostureSc3::~PostureSc3() { stop(); }

bool PostureSc3::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (index_ = 0; index_ < nbr_; index_++) {
    cameras_[index_]->start();
  }
  merger_->start();

  is_started_ = true;
  return true;
}

bool PostureSc3::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (index_ = 0; index_ < nbr_; index_++) {
    cameras_[index_]->stop();
  }
  merger_->stop();

  mesh_writer_.reset();
  rgb_writer_.reset();

  is_started_ = false;
  return true;
}

void PostureSc3::cb_frame_cloud(int index, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_started_) return;
  if (reload_calibration_) {
    calibration_reader_->loadCalibration(calibration_path_);
    merger_->setCalibration(calibration_reader_->getCalibrationParams());
  }
  merger_->setInputCloud(index, cloud);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  merger_->getCloud(temp_cloud);

  if (colorize_or_not_) {
    sol_->setInputCloud(temp_cloud);
    sol_->getMesh(intermediate_mesh_);
    return;
  }

  sol_->setInputCloud(temp_cloud);
  sol_->getMesh(output_);

  if (!mesh_writer_ ||
      output_.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
    mesh_writer_.reset();
    if (output_.size() >= 256) {
      // FIXME manual resize is not required since already done in shmdata
      mesh_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("mesh"), output_.size() * 2, string(POLYGONMESH_TYPE_BASE));
    } else {
      mesh_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("mesh"), 512, string(POLYGONMESH_TYPE_BASE));
    }

    if (!mesh_writer_) {
      warning("Unable to create mesh callback");
      return;
    }
  }

  mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>((void*)&output_, output_.size());
  mesh_writer_->bytes_written(output_.size());
}

void PostureSc3::cb_frame_rgb(std::vector<unsigned char>& image, int width, int heigth) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_started_) return;

  if (reload_calibration_) {
    calibration_reader_->loadCalibration(calibration_path_);
    merger_->setCalibration(calibration_reader_->getCalibrationParams());
  }

  std::vector<std::vector<unsigned char>> Images;
  Images.push_back(image);
  std::vector<std::vector<unsigned int>> Dims;
  Dims.push_back({(unsigned int)width, (unsigned int)heigth, 3});

  colorize_->setInput(intermediate_mesh_, Images, Dims);
  colorize_->getTexturedMesh(output_);
  texture_ = colorize_->getTexture((unsigned int&)width, (unsigned int&)heigth);

  if (!mesh_writer_ ||
      output_.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
    mesh_writer_.reset();
    if (output_.size() >= 256) {
      // FIXME manual resize is not required since already done in shmdata
      mesh_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("mesh"), output_.size() * 2, string(POLYGONMESH_TYPE_BASE));
    } else {
      mesh_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("mesh"), 512, string(POLYGONMESH_TYPE_BASE));
    }

    if (!mesh_writer_) {
      warning("Unable to create rgb callback");
      return;
    }
  }

  mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>((void*)&texture_, texture_.size());
  mesh_writer_->bytes_written(texture_.size());

  if (!rgb_writer_ || texture_.size() > rgb_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
    rgb_writer_.reset();
    if (texture_.size() >= 256) {
      // FIXME manual resize is not required since already done in shmdata
      rgb_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("rgb"), texture_.size() * 2, string(POLYGONMESH_TYPE_BASE));
    } else {
      rgb_writer_ = std::make_unique<ShmdataWriter>(
          this, make_file_name("rgb"), 512, string(POLYGONMESH_TYPE_BASE));
    }

    if (!rgb_writer_) {
      warning("Unable to create rgb callback");
      return;
    }
  }

  rgb_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>((void*)&texture_, texture_.size());
  rgb_writer_->bytes_written(texture_.size());

  std::cout << texture_.size() << std::endl;
  std::cout << texture_[0] << " " << texture_[100000] << " " << texture_[230000] << " "
            << std::endl;

  std::cout << output_.size() << std::endl;
  std::cout << output_[0] << " " << output_[100000] << " " << output_[230000] << " " << std::endl;
}

}  // namespace switcher
