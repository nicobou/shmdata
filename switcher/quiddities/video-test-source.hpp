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
#include "../gst/pipeliner.hpp"
#include "../gst/unique-gst-element.hpp"
#include "../quiddity/property/fraction.hpp"
#include "../quiddity/quiddity.hpp"
#include "../quiddity/startable.hpp"
#include "../shmdata/gst-tree-updater.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class VideoTestSource : public Quiddity, public quiddity::Startable {
 public:
  VideoTestSource(quiddity::Config&&);

 private:
  static const std::string kConnectionSpec;  //!< Shmdata specifications
  std::string shmpath_{};
  property::Selection<property::Fraction> resolutions_{
      {"3840x2160", "1920x1080", "1280x720", "800x600", "640x480", "320x240", "Custom"},
      {property::Fraction(3840, 2160),
       property::Fraction(1920, 1080),
       property::Fraction(1280, 720),
       property::Fraction(800, 600),
       property::Fraction(640, 480),
       property::Fraction(320, 240),
       property::Fraction(-1, -1)},
      1};  //!< width height
  property::prop_id_t resolutions_id_;
  int width_{1920};
  static const int kMaxWidth{4096};
  static const int kMinWidth{1};
  property::prop_id_t width_id_;
  int height_{1080};
  static const int kMaxHeight{4096};
  static const int kMinHeight{1};
  property::prop_id_t height_id_;
  property::Selection<property::Fraction> framerates_{
      {"60", "59.94", "50", "30", "29.97", "25", "24", "23.976"},
      {property::Fraction(60, 1),
       property::Fraction(5994, 100),
       property::Fraction(50, 1),
       property::Fraction(30, 1),
       property::Fraction(2997, 100),
       property::Fraction(25, 1),
       property::Fraction(24, 1),
       property::Fraction(23976, 1000)},  // or 2997/125
      3};                                 //!< frameratedefault to 30 fps
  property::prop_id_t framerates_id_;
  property::Selection<> formats_;
  const std::string cDefaultPixelFormat{"I420"};
  property::prop_id_t formats_id_;
  gst::UGstElem videotestsrc_{"videotestsrc"};
  gst::UGstElem capsfilter_{"capsfilter"};
  gst::UGstElem shmdatasink_{"shmdatasink"};
  std::unique_ptr<gst::Pipeliner> gst_pipeline_;
  std::unique_ptr<shmdata::GstTreeUpdater> shm_sub_{nullptr};
  bool start() final;
  bool stop() final;
  void update_caps();
};

}  // namespace quiddities
}  // namespace switcher
#endif
