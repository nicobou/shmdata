/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_GST_VIDEO_TIMELAPSE_H__
#define __SWITCHER_GST_VIDEO_TIMELAPSE_H__

#include <unordered_set>
#include <vector>
#include "../shmdata/gst-shm-tree-updater.hpp"
#include "./pipeliner.hpp"

namespace switcher {
namespace gst {
struct VideoTimelapseConfig {
  VideoTimelapseConfig(const std::string& orig_shmpath,
                          const std::string& image_path)
      :  // "for instance /tmp/img_%05d.jpg"
        orig_shmpath_(orig_shmpath),
        image_path_(image_path){};
  // config members:
  std::string orig_shmpath_{};
  std::string image_path_{};
  unsigned int framerate_num_{1};
  unsigned int framerate_denom_{1};
  unsigned int width_{0};
  unsigned int height_{0};
  unsigned int jpg_quality_{85};
  unsigned int max_files_{0};
};

class VideoTimelapse : public SafeBoolIdiom {
 public:
  using on_new_file_t = std::function<void(std::string&&)>;
  VideoTimelapse(const VideoTimelapseConfig& config,
                    Quiddity* quid,
                    on_new_file_t on_new_file);
  VideoTimelapse() = delete;
  ~VideoTimelapse() = default;
  VideoTimelapse(const VideoTimelapse&) = delete;
  VideoTimelapse& operator=(const VideoTimelapse&) = delete;

 private:
  VideoTimelapseConfig config_;
  on_new_file_t on_new_file_;
  // gst pipeline
  std::unique_ptr<Pipeliner> gst_pipeline_;
  std::unique_ptr<GstShmTreeUpdater> shmsrc_sub_{nullptr};

  // safe bool idiom
  bool is_valid_{false};
  bool safe_bool_idiom() const final { return is_valid_; };
};

}  // namespace gst
}  // namespace switcher
#endif
