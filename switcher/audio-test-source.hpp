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
#include "./gst-pipeliner.hpp"
#include "./quiddity.hpp"
#include "./startable-quiddity.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
class AudioTestSource: public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(AudioTestSource);
  AudioTestSource(const std::string &);
  ~AudioTestSource();
  AudioTestSource(const AudioTestSource &) = delete;
  AudioTestSource &operator=(const AudioTestSource &) = delete;

  bool start();
  bool stop();

 private:
  UGstElem audiotestsrc_{"audiotestsrc"};
  UGstElem shmdatasink_{"shmdatasink"};
  std::string caps_{};
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  std::atomic<bool> quit_{false};
  std::future<void> byte_monitor_{};
  bool init() final;
  static void on_caps_cb(GObject *gobject, GParamSpec *pspec, gpointer user_data);
  void byte_monitor();
};

}  // namespace switcher
#endif
