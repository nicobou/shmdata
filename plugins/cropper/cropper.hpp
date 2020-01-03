/*
 * This file is part of switcher-cropper.
 *
 * switcher-cropper is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_CROPPER_H__
#define __SWITCHER_CROPPER_H__

#include <string>

#include "switcher/gst/pipeliner.hpp"
#include "switcher/gst/unique-gst-element.hpp"
#include "switcher/gst/utils.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/shmdata/shmdata-connector.hpp"
#include "switcher/shmdata/shmdata-follower.hpp"
#include "switcher/utils/threaded-wrapper.hpp"

namespace switcher {
namespace quiddities {

using namespace quiddity;
class Cropper : public Quiddity {
 public:
  Cropper(quiddity::Config&&);
  ~Cropper() = default;
  Cropper(const Cropper&) = delete;
  Cropper& operator=(const Cropper&) = delete;

 private:
  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect();
  bool remake_elements();
  bool create_pipeline();
  bool can_sink_caps(const std::string& caps);

  gst::UGstElem shmsrc_{"shmdatasrc"};
  gst::UGstElem queue_element_{"queue"};
  gst::UGstElem cropper_element_{"videocrop"};
  gst::UGstElem scaler_element_{"videoscale"};
  gst::UGstElem shmsink_{"shmdatasink"};
  ShmdataConnector shmcntr_;
  std::unique_ptr<gst::Pipeliner> gst_pipeline_;
  std::string shmpath_to_crop_{};
  std::string shmpath_cropped_{};
  std::unique_ptr<ShmdataFollower> shmsrc_sub_{nullptr};
  std::unique_ptr<ShmdataFollower> shmsink_sub_{nullptr};
  std::string cur_caps_{};

  int left_{0};
  property::prop_id_t left_id_;
  int right_{0};
  property::prop_id_t right_id_;
  int top_{0};
  property::prop_id_t top_id_;
  int bottom_{0};
  property::prop_id_t bottom_id_;

  ThreadedWrapper<> async_this_{};
};
SWITCHER_DECLARE_PLUGIN(Cropper);
}  // namespace quiddities
}  // namespace switcher
#endif