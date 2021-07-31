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

#ifndef __SWITCHER_AUDIO_TEST_SOURCE_H__
#define __SWITCHER_AUDIO_TEST_SOURCE_H__

#include <atomic>
#include <future>
#include <memory>
#include "../gst/pipeliner.hpp"
#include "../gst/unique-gst-element.hpp"
#include "../quiddity/quiddity.hpp"
#include "../quiddity/startable.hpp"
#include "../shmdata/gst-tree-updater.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class AudioTestSource : public Quiddity, public Startable {
 public:
  AudioTestSource(quiddity::Config&&);

 private:
  static constexpr double kMaxFrequency = 20000.0;
  static const int kMaxChannels = 128;
  static const std::string kConnectionSpec;  //!< Shmdata specifications

  std::string shmpath_{};
  std::unique_ptr<gst::Pipeliner> gst_pipeline_;
  std::unique_ptr<shmdata::GstTreeUpdater> shm_sub_{nullptr};
  property::Selection<> sample_rate_{
      {"22050", "32000", "44100", "48000", "88200", "96000", "192000"}, 2};
  property::prop_id_t sample_rate_id_;
  double frequency_{440.0};
  property::prop_id_t frequency_id_;
  float volume_{0.5f};
  property::prop_id_t volume_id_;
  int channels_{1};
  property::prop_id_t channels_id_;
  property::Selection<> format_;
  property::prop_id_t format_id_;
  property::prop_id_t waveforms_id_{0};

  gst::UGstElem audiotestsrc_{"audiotestsrc"};
  gst::UGstElem capsfilter_{"capsfilter"};
  gst::UGstElem shmdatasink_{"shmdatasink"};
  bool start() final;
  bool stop() final;
  void update_caps();
};

}  // namespace quiddities
}  // namespace switcher
#endif
