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

#include <memory>
#include <future>
#include <atomic>
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/unique-gst-element.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"

namespace switcher {
class AudioTestSource: public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(AudioTestSource);
  AudioTestSource(const std::string &);
  ~AudioTestSource() = default;
  AudioTestSource(const AudioTestSource &) = delete;
  AudioTestSource &operator=(const AudioTestSource &) = delete;

 private:
  std::string shmpath_{};
  UGstElem audiotestsrc_{"audiotestsrc"};
  UGstElem shmdatasink_{"shmdatasink"};
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};
  bool start() final;
  bool stop() final;
  bool init() final;
};

}  // namespace switcher
#endif
