#ifndef POSTURE_SCAN3DGPU_H
#define POSTURE_SCAN3DGPU_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

#include <boost/make_shared.hpp>

namespace switcher {
class PostureScan3DGPU : public Quiddity, public StartableQuiddity {
 public:
  PostureScan3DGPU(QuiddityConfiguration&&);
  PostureScan3DGPU(const PostureScan3DGPU&) = delete;
  ~PostureScan3DGPU();

  bool start();
  bool stop();

 private:
  bool compress_mesh_{false};
  bool compress_multicore_{true};
  std::string calibration_path_{"default.kvc"};

  int camera_nbr_{1};
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

  std::vector<double> grid_size_{1.0, 1.0, 1.0};
  PropertyBase::prop_id_t grid_size_x_id_, grid_size_y_id_, grid_size_z_id_;
  uint32_t grid_resolution_{64};

  int kernel_filter_size_{5};
  float kernel_spatial_sigma_{5.0};
  float kernel_value_sigma_{100.0};
  int hole_filling_iterations_{1};

  bool refine_mesh_{false};
  float refine_mesh_point_separation_{0.1f};
  int refine_mesh_max_neighbours_{8};

  std::mutex camera_mutex_;
  std::vector<std::unique_ptr<posture::ZCamera>> cameras_{};
  std::unique_ptr<posture::SolidifyGPU> solidifyGPU_{nullptr};
  std::unique_ptr<posture::ColorizeGL> colorize_{nullptr};
  std::unique_ptr<posture::SculptGL> sculpt_{nullptr};
  std::unique_ptr<posture::Register> register_{nullptr};
  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};

  PropertyBase::prop_id_t register_id_;
  bool reload_calibration_{false};
  bool improve_registering_{false};
  std::thread registering_thread_{};
  std::atomic_bool is_registering_{false};

  std::vector<bool> cameras_updated_{};
  std::vector<std::vector<uint8_t>> images_{};
  std::vector<std::vector<uint32_t>> images_dims_{};

  std::atomic_bool update_loop_started_{false};
  std::atomic_bool update_wanted_{false};
  std::mutex update_mutex_;
  std::condition_variable update_cv_;
  std::thread update_thread_;

  std::unique_ptr<ShmdataWriter> mesh_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> texture_writer_{nullptr};

  void update_loop();
  void reset_solidify();

  bool all(const std::vector<bool>& status);
  void zero(std::vector<bool>& status);
  void cb_frame_cloud(int index, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
  void cb_frame_depth(int index, std::vector<unsigned char>& depth, int width, int height);
  void cb_frame_RGB(int index, std::vector<unsigned char>& rgb, int width, int height);
};

SWITCHER_DECLARE_PLUGIN(PostureScan3DGPU);
}  // namespace switcher
#endif  // POSTURE_SCAN3DGPU_H
