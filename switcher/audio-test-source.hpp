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
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/unique-gst-element.hpp"

namespace switcher {
class AudioTestSource : public Quiddity, public StartableQuiddity {
 public:
  AudioTestSource(QuiddityConfiguration&&);
  ~AudioTestSource() = default;
  AudioTestSource(const AudioTestSource&) = delete;
  AudioTestSource& operator=(const AudioTestSource&) = delete;

 private:
  static constexpr double kMaxFrequency = 20000.0;
  static const int kMaxChannels = 128;

  std::string shmpath_{};
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  Selection<> sample_rate_{{"22050", "32000", "44100", "48000", "88200", "96000", "192000"}, 2};
  PContainer::prop_id_t sample_rate_id_;
  double frequency_{440.0};
  PContainer::prop_id_t frequency_id_;
  float volume_{0.5f};
  PContainer::prop_id_t volume_id_;
  int channels_{1};
  PContainer::prop_id_t channels_id_;
  Selection<> format_;
  PContainer::prop_id_t format_id_;
  PContainer::prop_id_t waveforms_id_{0};

  UGstElem audiotestsrc_{"audiotestsrc"};
  UGstElem capsfilter_{"capsfilter"};
  UGstElem shmdatasink_{"shmdatasink"};
  bool start() final;
  bool stop() final;
  void update_caps();
};

}  // namespace switcher
#endif
