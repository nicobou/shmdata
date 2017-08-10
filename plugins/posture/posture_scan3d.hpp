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

#ifndef __SWITCHER_POSTURE_SCAN3D_H__
#define __SWITCHER_POSTURE_SCAN3D_H__

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class PostureSc3 : public Quiddity, public StartableQuiddity {
 public:
  PostureSc3(QuiddityConfiguration&&);
  PostureSc3(const PostureSc3&) = delete;
  ~PostureSc3();

  bool start();
  bool stop();

 private:

  static int get_output_mesh(std::vector<unsigned char>);
  void cb_frame_cloud(int index, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
  void cb_frame_rgb(std::vector<unsigned char>& image, int width, int heigth);

  std::vector<std::shared_ptr<posture::ZCamera>> cameras_{};
  std::unique_ptr<posture::PointCloudMerger> merger_{nullptr};
  std::unique_ptr<posture::Solidify> sol_{nullptr};
  std::unique_ptr<posture::Colorize> colorize_{nullptr};

  std::vector<unsigned char> output_{};

  mutable std::mutex mutex_{};

  std::unique_ptr<ShmdataWriter> mesh_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> rgb_writer_{nullptr};

  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};

  int index_{0};
  int nbr_{1};
  std::string calibration_path_{"default.kvc"};
  int grid_res_{3};
  bool reload_calibration_{false};
  bool colorize_or_not_{false};

  bool is_started_{false};

  pcl::PolygonMesh::Ptr intermediate_mesh_{nullptr};
  std::vector<unsigned char> texture_{};
};

SWITCHER_DECLARE_PLUGIN(PostureSc3);
}  // namespace switcher
#endif
