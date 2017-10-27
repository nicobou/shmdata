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

#include "./video-test-source.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VideoTestSource,
                                     "videotestsrc",
                                     "Video Pattern",
                                     "video",
                                     "writer",
                                     "Creates a test video stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

VideoTestSource::VideoTestSource(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      resolutions_id_(pmanage<MPtr(&PContainer::make_selection<Fraction>)>(
          "resolution",
          [this](const IndexOrName& val) {
            resolutions_.select(val);
            if (resolutions_.get_current() == "Custom") {
              pmanage<MPtr(&PContainer::enable)>(width_id_);
              pmanage<MPtr(&PContainer::enable)>(height_id_);
              return true;
            }
            auto fract = resolutions_.get_attached();
            pmanage<MPtr(&PContainer::set<int>)>(width_id_, fract.numerator());
            pmanage<MPtr(&PContainer::set<int>)>(height_id_, fract.denominator());
            static const std::string why_disconnected =
                "This property is available only with custom resolution";
            pmanage<MPtr(&PContainer::disable)>(width_id_, why_disconnected);
            pmanage<MPtr(&PContainer::disable)>(height_id_, why_disconnected);
            return true;
          },
          [this]() { return resolutions_.get(); },
          "Resolutions",
          "Select resolutions",
          resolutions_)),
      width_id_(pmanage<MPtr(&PContainer::make_int)>("width",
                                                     [this](int val) {
                                                       width_ = val;
                                                       return true;
                                                     },
                                                     [this]() { return width_; },
                                                     "Width",
                                                     "Set Video Width",
                                                     width_,
                                                     kMinWidth,
                                                     kMaxWidth)),
      height_id_(pmanage<MPtr(&PContainer::make_int)>("height",
                                                      [this](int val) {
                                                        height_ = val;
                                                        return true;
                                                      },
                                                      [this]() { return height_; },
                                                      "Height",
                                                      "Set Video Height",
                                                      height_,
                                                      kMinHeight,
                                                      kMaxHeight)),
      framerates_id_(pmanage<MPtr(&PContainer::make_selection<Fraction>)>(
          "framerate",
          [this](const IndexOrName& val) {
            framerates_.select(val);
            return true;
          },
          [this]() { return framerates_.get(); },
          "Video Framerate",
          "Select the video framerate",
          framerates_)),
      formats_(Selection<>(
          GstUtils::get_gst_element_capability_as_list("videotestsrc", "format", GST_PAD_SRC), 0)),
      formats_id_(pmanage<MPtr(&PContainer::make_selection<>)>("format",
                                                               [this](const IndexOrName& val) {
                                                                 formats_.select(val);
                                                                 return true;
                                                               },
                                                               [this]() { return formats_.get(); },
                                                               "Video Pixel Format",
                                                               "Select the pixel video format",
                                                               formats_)) {
  // We do this so that width and height properties states are correct.
  pmanage<MPtr(&PContainer::set_to_current)>(resolutions_id_);

  init_startable(this);

  if (!videotestsrc_ || !capsfilter_ || !shmdatasink_) {
    is_valid_ = false;
    return;
  }
  shmpath_ = make_file_name("video");
  g_object_set(G_OBJECT(videotestsrc_.get_raw()), "is-live", TRUE, nullptr);
  g_object_set(G_OBJECT(shmdatasink_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  update_caps();
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmdatasink_.get_raw(),
                   capsfilter_.get_raw(),
                   videotestsrc_.get_raw(),
                   nullptr);
  gst_element_link_many(
      videotestsrc_.get_raw(), capsfilter_.get_raw(), shmdatasink_.get_raw(), nullptr);
  pmanage<MPtr(&PContainer::push)>(
      "pattern", GPropToProp::to_prop(G_OBJECT(videotestsrc_.get_raw()), "pattern"));
}

void VideoTestSource::update_caps() {
  auto framerate = framerates_.get_attached();
  auto caps_str = std::string("video/x-raw, format=") + formats_.get_current() + ", width=" +
                  std::to_string(width_) + ", height=" + std::to_string(height_) + ", framerate=" +
                  std::to_string(framerate.numerator()) + "/" +
                  std::to_string(framerate.denominator()) +
                  ", pixel-aspect-ratio=1/1, interlace-mode=progressive";
  GstCaps* caps = gst_caps_from_string(caps_str.c_str());
  On_scope_exit { gst_caps_unref(caps); };
  g_object_set(G_OBJECT(capsfilter_.get_raw()), "caps", caps, nullptr);
}

bool VideoTestSource::start() {
  if (!gst_pipeline_) return false;
  shm_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmdatasink_.get_raw(),
      [this](const std::string& caps) {
        this->graft_tree(
            ".shmdata.writer." + shmpath_,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.writer." + shmpath_));
  update_caps();
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  gst_pipeline_->play(true);
  pmanage<MPtr(&PContainer::replace)>(
      pmanage<MPtr(&PContainer::get_id)>("pattern"),
      GPropToProp::to_prop(G_OBJECT(videotestsrc_.get_raw()), "pattern"));
  pmanage<MPtr(&PContainer::disable)>(width_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(height_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(resolutions_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(framerates_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&PContainer::disable)>(formats_id_, disabledWhenStartedMsg);
  return true;
  }

  bool VideoTestSource::stop() {
    shm_sub_.reset(nullptr);
    prune_tree(".shmdata.writer." + shmpath_);
    if (!UGstElem::renew(videotestsrc_, {"is-live", "pattern"}) ||
        !UGstElem::renew(shmdatasink_, {"socket-path"}) || !UGstElem::renew(capsfilter_)) {
      warning("error initializing gst element for videotestsrc");
      gst_pipeline_.reset();
      return false;
    }
    pmanage<MPtr(&PContainer::replace)>(
        pmanage<MPtr(&PContainer::get_id)>("pattern"),
        GPropToProp::to_prop(G_OBJECT(videotestsrc_.get_raw()), "pattern"));
    gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
    gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                     shmdatasink_.get_raw(),
                     capsfilter_.get_raw(),
                     videotestsrc_.get_raw(),
                     nullptr);
    gst_element_link_many(
        videotestsrc_.get_raw(), capsfilter_.get_raw(), shmdatasink_.get_raw(), nullptr);
    pmanage<MPtr(&PContainer::enable)>(resolutions_id_);
    // This way it will let the setter manage the state of width/height property.
    pmanage<MPtr(&PContainer::set_to_current)>(resolutions_id_);
    pmanage<MPtr(&PContainer::enable)>(framerates_id_);
    pmanage<MPtr(&PContainer::enable)>(formats_id_);
    return true;
  }

}  // namespace switcher
