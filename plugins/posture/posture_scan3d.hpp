
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
  static void set_input_camera(const int camera_nbr, void* user_data);
  static int get_input_camera(void* context);
  static int get_output_mesh(std::vector<unsigned char>);
  void cb_frame_cloud(int index,
                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *nbr_props_ {nullptr};

  std::vector<std::shared_ptr<posture::ZCamera>> cameras_ {};
  std::unique_ptr<posture::PointCloudMerger> merger_ {nullptr};
  std::unique_ptr<posture::Solidify> sol_ {nullptr};

  std::vector<unsigned char> output_ {};

  mutable std::mutex mutex_;

  std::unique_ptr<ShmdataWriter> mesh_writer_ {nullptr};

  int index_ {0};
  int nbr_ {0};
};

SWITCHER_DECLARE_PLUGIN(PostureSc3);
}  // namespace switcher

#endif
