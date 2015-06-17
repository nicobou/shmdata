
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
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class PostureSc3:public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureSc3);
  PostureSc3(const std::string &);
  PostureSc3(const PostureSc3 &) = delete;
  ~PostureSc3();

  bool start();
  bool stop();

 private:
  bool init() final;

  static int get_input_camera(void* context);
  static void set_input_camera(const int camera_nbr, void* user_data);
  static const gchar *get_calibration_path(void *user_data);
  static void set_calibration_path(const gchar *name, void *user_data);
  static const gchar *get_devices_path(void *user_data);
  static void set_devices_path(const gchar *name, void *user_data);
  static int get_grid_props(void *user_data);
  static void set_grid_props(const int resolution, void *user_data);
  static int get_reload_calibration(void *user_data);
  static void set_reload_calibration(const int reload, void *user_data);
  static int get_colorize(void *user_data);
  static void set_colorize(const int assert_colorize, void *user_data);

  static int get_output_mesh(std::vector<unsigned char>);
  void cb_frame_cloud(int index,
                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
  void cb_frame_rgb(std::vector<unsigned char>& image, int width, int heigth);

  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *nbr_props_ {nullptr};
  GParamSpec *calibration_path_props_ {nullptr};
  GParamSpec *devices_path_props_ {nullptr};
  GParamSpec *grid_res_props_ {nullptr};
  GParamSpec *reload_calibration_props_ {nullptr};
  GParamSpec *colorize_props_ {nullptr};

  std::vector<std::shared_ptr<posture::ZCamera>> cameras_ {};
  std::unique_ptr<posture::PointCloudMerger> merger_ {nullptr};
  std::unique_ptr<posture::Solidify> sol_ {nullptr};
  std::unique_ptr<posture::Colorize> colorize_ {nullptr};

  std::vector<unsigned char> output_ {};

  mutable std::mutex mutex_ {};

  std::unique_ptr<ShmdataWriter> mesh_writer_ {nullptr};
  std::unique_ptr<ShmdataWriter> rgb_writer_ {nullptr};

  int index_ {0};
  int nbr_ {1};
  std::string calibration_path_ {"default.kvc"};
  std::string devices_path_ {"devices.xml"};
  int grid_res_ {3};
  bool reload_calibration_ {false};
  bool colorize_or_not_ {false};

  bool is_started_ {false};

  pcl::PolygonMesh::Ptr intermediate_mesh_ {nullptr};
  std::vector<unsigned char> texture_ {};
};

SWITCHER_DECLARE_PLUGIN(PostureSc3);
}  // namespace switcher

#endif
