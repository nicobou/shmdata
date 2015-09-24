/*
 * This file is part of switcher-v4l2.
 *
 * switcher-v4l2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SWITCHER_V4L2SRC_H__
#define __SWITCHER_V4L2SRC_H__

#include <memory>
#include "switcher/std2.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/unique-gst-element.hpp"
#include "switcher/gst-video-codec.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/gst-video-codec.hpp"

namespace switcher {
class V4L2Src: public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(V4L2Src);
  V4L2Src(const std::string &);
  ~V4L2Src() = default;
  V4L2Src(const V4L2Src &) = delete;
  V4L2Src &operator=(const V4L2Src &) = delete;

  // use "NONE" for used arguments
  /* bool capture_full (const char *device_file_path,  */
  /*        const char *width, */
  /*        const char *height, */
  /*        const char *framerate_numerator, */
  /*        const char *framerate_denominator, */
  /*        const char *tv_standard); */

  static bool is_v4l_device(const char *file);

  bool inspect_file_device(const char *file_path);
  bool check_folder_for_v4l2_devices(const char *dir_path);

 private:
  bool start() final;
  bool stop() final;
  bool configure_capture();

  UGstElem v4l2src_{"v4l2src"};
  UGstElem capsfilter_{"capsfilter"};
  UGstElem shmsink_{"shmdatasink"};
  std::string shmpath_{};
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};
  std::unique_ptr<GstVideoCodec> codecs_{nullptr};

  typedef struct {
    std::string card_{};
    std::string file_device_{};
    std::string bus_info_{};
    std::string driver_{};
    std::vector<std::pair<std::string/*name*/,
                          std::string/*description*/>>pixel_formats_{};
    std::vector<std::pair<std::string/*width*/,
                          std::string/*height*/>>frame_size_discrete_{};
    gint frame_size_stepwise_max_width_{0};
    gint frame_size_stepwise_min_width_{0};
    gint frame_size_stepwise_step_width_{0};
    gint frame_size_stepwise_max_height_{0};
    gint frame_size_stepwise_min_height_{0};
    gint frame_size_stepwise_step_height_{0};
    std::vector<std::string> tv_standards_{};
    std::vector<std::pair<std::string/*numerator*/,
                          std::string/*denominator*/>>frame_interval_discrete_{};
    gint frame_interval_stepwise_min_numerator_{0};
    gint frame_interval_stepwise_min_denominator_{0};
    gint frame_interval_stepwise_max_numerator_{0};
    gint frame_interval_stepwise_max_denominator_{0};
    gint frame_interval_stepwise_step_numerator_{0};
    gint frame_interval_stepwise_step_denominator_{0};
  } CaptureDescription;

  bool remake_elements();
  void update_capture_device();
  void update_device_specific_properties(gint device_enum_id);
  void update_discrete_resolution(const CaptureDescription &descr);
  void update_width_height(const CaptureDescription &descr);
  void update_tv_standard(const CaptureDescription &descr);
  void update_discrete_framerate(const CaptureDescription &cap_descr);
  void update_framerate_numerator_denominator(const CaptureDescription &cap_descr);
  void update_pixel_format(const CaptureDescription &cap_descr);

  static std::string pixel_format_to_string(unsigned pf_id);

  // devices list:
  Selection devices_enum_{{"none"}, 0};
  PContainer::prop_id_t devices_id_{0};

  // pixet format
  Selection pixel_format_enum_{{"none"}, 0};
  PContainer::prop_id_t pixel_format_id_{0};

  // resolution enum and select for the currently selected device,
  // this is updated when selecting an other device
  Selection resolutions_enum_{{"none"}, 0};
  PContainer::prop_id_t resolutions_id_{0};
  // width height for the currently selected device
  gint width_{0};
  PContainer::prop_id_t width_id_{0};
  gint height_{0};
  PContainer::prop_id_t height_id_{0};

  // tv standard enum and select for the currently selected device,
  // this is updated when selecting an other device
  Selection tv_standards_enum_{{"none"}, 0};
  PContainer::prop_id_t tv_standards_id_{0};

  // framerate enum and select for the currently selected device,
  // this is updated when selecting an other device
  Selection framerates_enum_{{"none"}, 0};
  PContainer::prop_id_t framerates_enum_id_{0};

  // width height for the currently selected device
  Fraction framerate_{30, 1};
  PContainer::prop_id_t framerate_id_{0};

  // copy/paste from gstv4l2object.c for converting v4l2 pixel formats
  // to GstStructure (and then caps)
  static GstStructure *gst_v4l2_object_v4l2fourcc_to_structure (guint32 fourcc);
  std::vector<CaptureDescription> capture_devices_{};  // FIXME should be static
  bool init() final;
};

SWITCHER_DECLARE_PLUGIN(V4L2Src);

}  // namespace switcher
#endif
