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

#ifndef __SWITCHER_VIDEO_TEST_SOURCE_H__
#define __SWITCHER_VIDEO_TEST_SOURCE_H__

#include <memory>
#include <vector>
#include "switcher/fraction.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/unique-gst-element.hpp"

namespace switcher {
class VideoTestSource : public Quiddity, public StartableQuiddity {
 public:
  VideoTestSource(QuiddityConfiguration&&);
  ~VideoTestSource() = default;
  VideoTestSource(const VideoTestSource&) = delete;
  VideoTestSource& operator=(const VideoTestSource&) = delete;

 private:
  std::string shmpath_{};
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  // width height
  Selection<Fraction> resolutions_{
      {"3840x2160", "1920x1080", "1280x720", "800x600", "640x480", "320x240", "Custom"},
      {Fraction(3840, 2160),
       Fraction(1920, 1080),
       Fraction(1280, 720),
       Fraction(800, 600),
       Fraction(640, 480),
       Fraction(320, 240),
       Fraction(-1, -1)},
      1};
  PContainer::prop_id_t resolutions_id_;
  int width_{1920};
  static const int kMaxWidth{4096};
  static const int kMinWidth{1};
  PContainer::prop_id_t width_id_;
  int height_{1080};
  static const int kMaxHeight{4096};
  static const int kMinHeight{1};
  PContainer::prop_id_t height_id_;
  // framerate
  Selection<Fraction> framerates_{{"60", "59.94", "50", "30", "29.97", "25", "24", "23.976"},
                                  {Fraction(60, 1),
                                   Fraction(5994, 100),
                                   Fraction(50, 1),
                                   Fraction(30, 1),
                                   Fraction(2997, 100),
                                   Fraction(25, 1),
                                   Fraction(24, 1),
                                   Fraction(23976, 1000)},  // or 2997/125
                                  3};                       // default to 30 fps
  PContainer::prop_id_t framerates_id_;
  // formats
  Selection<> formats_;
  PContainer::prop_id_t formats_id_;
  // gst elements
  UGstElem videotestsrc_{"videotestsrc"};
  UGstElem capsfilter_{"capsfilter"};
  UGstElem shmdatasink_{"shmdatasink"};
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};
  bool start() final;
  bool stop() final;
  void update_caps();
};

}  // namespace switcher
#endif
