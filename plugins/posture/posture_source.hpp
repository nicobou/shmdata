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

#ifndef __SWITCHER_POSTURE_SRC_H__
#define __SWITCHER_POSTURE_SRC_H__

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class PostureSrc : public Quiddity, public StartableQuiddity {
 public:
  PostureSrc(QuiddityConfiguration&&);
  ~PostureSrc();
  PostureSrc(const PostureSrc&) = delete;
  PostureSrc& operator=(const PostureSrc&) = delete;

  bool start();
  bool stop();

 private:
  // If true, does not connect to a camera
  // but outputs random cloud and mesh
  bool random_data_{false};
  bool do_random_data_{false};
  std::thread random_data_thread_{};

  double rgb_focal_{0.0};
  PContainer::prop_id_t rgb_focal_id_{0};
  double depth_focal_{0.0};
  PContainer::prop_id_t depth_focal_id_{0};
  std::string calibration_path_{"default.kvc"};
  unsigned int device_index_{0};
  bool capture_ir_{false};
  bool build_mesh_{false};
  int build_mesh_edge_length_{1};
  PContainer::prop_id_t build_mesh_edge_length_id_{0};
  bool compress_cloud_{false};
  bool reload_calibration_{false};
  bool downsample_{false};
  double downsample_resolution_{0.1};
  PContainer::prop_id_t downsample_resolution_id_{0};
  bool filter_outliers_{false};
  int filter_mean_k_{8};
  PContainer::prop_id_t filter_mean_k_id_{0};
  double filter_stddev_mul_{1.0};
  PContainer::prop_id_t filter_stddev_mul_id_{0};
  int bilateral_filter_kernel_size_{5};
  double bilateral_filter_sigma_pos_{5.f};
  double bilateral_filter_sigma_value_{1000.f};
  int bilateral_filter_iterations_{1};
  int hole_filling_kernel_size_{1};
  int hole_filling_iterations_{1};

  Selection<> capture_modes_enum_{{"Default mode",
                                   "SXGA 15Hz",
                                   "VGA 30Hz",
                                   "VGA 25Hz",
                                   "QVGA 25Hz",
                                   "QVGA 30Hz",
                                   "QVGA 60Hz",
                                   "QQVGA 25Hz",
                                   "QQVGA 30Hz",
                                   "QQVGA 60Hz"},
                                  0};

  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};
  std::unique_ptr<posture::ZCamera> zcamera_{nullptr};

  std::unique_ptr<ShmdataWriter> cloud_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> mesh_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> depth_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> rgb_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> ir_writer_{nullptr};

  bool cloud_compressed_{false};

  int depth_width_{0}, depth_height_{0};
  int rgb_width_{0}, rgb_height_{0};
  int ir_width_{0}, ir_height_{0};
  posture::ZCamera::CaptureFormat rgb_format_{posture::ZCamera::CaptureFormat::RGB};


  static void cb_frame_cloud(void* context, const std::vector<char>&& data);
  static void cb_frame_mesh(void* context, std::vector<unsigned char>&& data);
  static void cb_frame_depth(void* context,
                             const std::vector<unsigned char>& data,
                             int width,
                             int height);
  static void cb_frame_rgb(void* context,
                           const std::vector<unsigned char>& data,
                           int width,
                           int height);
  static void cb_frame_ir(void* context,
                          const std::vector<unsigned char>& data,
                          int width,
                          int height);

  // For debug purpose: generates random data
  void generateRandomData();
};

SWITCHER_DECLARE_PLUGIN(PostureSrc);
}  // namespace switcher
#endif
