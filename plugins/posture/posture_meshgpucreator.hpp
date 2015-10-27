#ifndef POSTURE_MESHGPUCREATOR_H
#define POSTURE_MESHGPUCREATOR_H

#include <deque>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/startable-quiddity.hpp"

#include <boost/make_shared.hpp>

namespace switcher {
  class PostureMeshGPUCreator:public Quiddity, public StartableQuiddity {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureMeshGPUCreator);
    PostureMeshGPUCreator (const std::string &);
    PostureMeshGPUCreator(const PostureMeshGPUCreator &) = delete;
    ~PostureMeshGPUCreator();

    bool start();
    bool stop();

  private:
    std::string calibration_path_ {"default.kvc"};
    std::string devices_path_ {"devices.xml"};
    double resolution_ {0.05};
    double size_ {1.0f};
    double sizeX_ {1.0f};
    double sizeY_ {1.0f};
    double sizeZ_ {1.0f};
    PContainer::prop_id_t sizeX_id_{0};
    PContainer::prop_id_t sizeY_id_{0};
    PContainer::prop_id_t sizeZ_id_{0};
    int nbr_cam_ {1};
    bool save_mesh_ {false};
    bool is_started_ {false};
    std::vector<std::unique_ptr<posture::ZCamera>> cameras_ {};
    std::unique_ptr<posture::DepthMapToMesh> mesh_creator_ {nullptr};
    std::vector<unsigned char> output_ {};
    std::mutex mutex_ {};

    std::unique_ptr<ShmdataWriter> mesh_writer_ {nullptr};
    std::vector<std::unique_ptr<ShmdataWriter>> rgb_writers_ {};

    std::map<int, std::vector<unsigned char>> depth_stock_ {};
    std::mutex stock_mutex_ {};
  
    bool init() final;
    void cb_frame_depth(int index, std::vector<unsigned char>& depth, int width, int height);
    void cb_frame_cloud(int index, std::vector<char>& cloud);
    void cb_frame_RGB(int index, std::vector<unsigned char>& rgb, int width, int height);

//    posture::Display _disp;
  };

  SWITCHER_DECLARE_PLUGIN(PostureMeshGPUCreator);
}  // namespace switcher
#endif  // POSTURE_MESHGPUCREATOR_H
