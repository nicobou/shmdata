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

#include <vector>
#include <unordered_set>
#include "switcher/unique-gst-element.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/shmdata-utils.hpp"


namespace switcher {
class quiddity;

struct GstVideoTimelapseConfig{
  GstVideoTimelapseConfig(const std::string &orig_shmpath,
                         const std::string &image_path): // "for instance /tmp/img_%05d.jpg"
      orig_shmpath_(orig_shmpath),
      image_path_(image_path){};
  // config members:
  std::string orig_shmpath_{};
  std::string image_path_{};
  unsigned int framerate_num_{1};
  unsigned int framerate_denom_{1};
};

class GstVideoTimelapse {
 public:
  GstVideoTimelapse(Quiddity *quid,
                   const GstVideoTimelapseConfig &config);
  GstVideoTimelapse() = delete;
  ~GstVideoTimelapse() = default;
  GstVideoTimelapse(const GstVideoTimelapse &) = delete;
  GstVideoTimelapse &operator=(const GstVideoTimelapse &) = delete;

 private:
  Quiddity *quid_;
  GstVideoTimelapseConfig config_;
  // gst pipeline
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  // UGstElem shmsrc_{"shmdatasrc"};
  std::unique_ptr<GstShmdataSubscriber> shmsrc_sub_{nullptr};
};

}  // namespace switcher
#endif
