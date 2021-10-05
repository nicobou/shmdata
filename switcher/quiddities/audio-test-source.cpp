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

#include "./audio-test-source.hpp"
#include <thread>
#include "../quiddity/property/gprop-to-prop.hpp"
#include "../utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AudioTestSource,
                                     "audiotestsrc",
                                     "Audio Test Source",
                                     "Creates audio test signals",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string AudioTestSource::kConnectionSpec(R"(
{
"writer":
  [
    {
      "label": "audio",
      "description": "The audio test signal",
      "can_do": ["audio/x-raw"]
    }
  ]
}
)");

AudioTestSource::AudioTestSource(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf), {kConnectionSpec}),
      quiddity::Startable(this),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)),
      sample_rate_id_(pmanage<MPtr(&property::PBag::make_selection<>)>(
          "sample_rate",
          [this](const quiddity::property::IndexOrName& val) {
            sample_rate_.select(val);
            return true;
          },
          [this]() { return sample_rate_.get(); },
          "Sample rate",
          "List of supported sample rates",
          sample_rate_)),
      frequency_id_(pmanage<MPtr(&property::PBag::make_double)>(
          "frequency",
          [this](const double& val) {
            frequency_ = val;
            g_object_set(G_OBJECT(audiotestsrc_.get_raw()), "freq", frequency_, nullptr);
            return true;
          },
          [this]() { return frequency_; },
          "Frequency",
          "Set sound frequency",
          440.0,
          1.0,
          kMaxFrequency)),
      volume_id_(pmanage<MPtr(&property::PBag::make_float)>(
          "volume",
          [this](const float& val) {
            volume_ = val;
            g_object_set(G_OBJECT(audiotestsrc_.get_raw()), "volume", volume_, nullptr);
            return true;
          },
          [this]() { return volume_; },
          "Volume",
          "Set sound volume",
          0.5f,
          0.0f,
          1.0f)),
      channels_id_(pmanage<MPtr(&property::PBag::make_int)>(
          "channels",
          [this](const int& val) {
            channels_ = val;
            return true;
          },
          [this]() { return channels_; },
          "Number of channels",
          "Set number of channels",
          1,
          1,
          kMaxChannels)),
      format_(property::Selection<>(
          gst::utils::get_gst_element_capability_as_list("audiotestsrc", "format", GST_PAD_SRC),
          0)),
      format_id_(pmanage<MPtr(&property::PBag::make_selection<>)>(
          "format",
          [this](const quiddity::property::IndexOrName& val) {
            format_.select(val);
            return true;
          },
          [this]() { return format_.get(); },
          "Format",
          "List of supported sound formats",
          format_)) {
  if (!audiotestsrc_ || !capsfilter_ || !shmdatasink_) {
    is_valid_ = false;
    return;
  }

  shmpath_ = claw_.get_shmpath_from_writer_label("audio");
  g_object_set(G_OBJECT(audiotestsrc_.get_raw()), "is-live", TRUE, nullptr);
  g_object_set(G_OBJECT(audiotestsrc_.get_raw()), "samplesperbuffer", 512, nullptr);
  g_object_set(G_OBJECT(shmdatasink_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  auto extra_caps = get_quiddity_caps();
  g_object_set(G_OBJECT(shmdatasink_.get_raw()), "extra-caps-properties", extra_caps.c_str(), nullptr);
  waveforms_id_ = pmanage<MPtr(&property::PBag::push)>(
      "wave", quiddity::property::to_prop(G_OBJECT(audiotestsrc_.get_raw()), "wave"));
}

bool AudioTestSource::start() {
  if (!gst_pipeline_) {
    warning("BUG: gst_pipeline failed to be created, something went very wrong! (audiotestsrc)");
    return false;
  }

  shm_sub_ = std::make_unique<shmdata::GstTreeUpdater>(
      this, shmdatasink_.get_raw(), shmpath_, shmdata::GstTreeUpdater::Direction::writer);
  update_caps();
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   audiotestsrc_.get_raw(),
                   capsfilter_.get_raw(),
                   shmdatasink_.get_raw(),
                   nullptr);
  gst_element_link_many(
      audiotestsrc_.get_raw(), capsfilter_.get_raw(), shmdatasink_.get_raw(), nullptr);

  pmanage<MPtr(&property::PBag::disable)>(format_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(channels_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(sample_rate_id_, disabledWhenStartedMsg);
  gst_pipeline_->play(true);

  return true;
}

bool AudioTestSource::stop() {
  shm_sub_.reset();

  pmanage<MPtr(&property::PBag::enable)>(format_id_);
  pmanage<MPtr(&property::PBag::enable)>(channels_id_);
  pmanage<MPtr(&property::PBag::enable)>(sample_rate_id_);

  if (!gst::UGstElem::renew(audiotestsrc_, {"is-live", "samplesperbuffer"}) ||
      !gst::UGstElem::renew(capsfilter_) || !gst::UGstElem::renew(shmdatasink_, {"socket-path", "extra-caps-properties"})) {
    warning("error initializing gst element for audiotestsrc");
    gst_pipeline_.reset();
    return false;
  }

  pmanage<MPtr(&property::PBag::replace)>(
      waveforms_id_, quiddity::property::to_prop(G_OBJECT(audiotestsrc_.get_raw()), "wave"));
  gst_pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);

  return true;
}

void AudioTestSource::update_caps() {
  std::string str_caps = "audio/x-raw,format=" + format_.get_current() + ",channels=" +
                         std::to_string(channels_) + ",rate=" + sample_rate_.get_current();
  debug("caps: %", str_caps);
  GstCaps* caps = gst_caps_from_string(str_caps.c_str());
  On_scope_exit { gst_caps_unref(caps); };
  g_object_set(G_OBJECT(capsfilter_.get_raw()), "caps", caps, nullptr);
}

}  // namespace quiddities
}  // namespace switcher
