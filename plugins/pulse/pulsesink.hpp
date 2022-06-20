/*
 * This file is part of switcher-pulse.
 *
 * switcher-pulse is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_PULSE_SINK_H__
#define __SWITCHER_PULSE_SINK_H__

#include <pulse/glib-mainloop.h>
#include <pulse/pulseaudio.h>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "switcher/gst/glibmainloop.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/shmdata/caps/audio-caps.hpp"
#include "switcher/shmdata/follower.hpp"
#include "switcher/utils/audio-resampler.hpp"
#include "switcher/utils/audio-ring-buffer.hpp"
#include "switcher/utils/drift-observer.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class PulseSink : public Quiddity {
 public:
  PulseSink(quiddity::Config&&);
  ~PulseSink();
  PulseSink(const PulseSink&) = delete;
  PulseSink& operator=(const PulseSink&) = delete;

 private:
  typedef struct {
    std::string name_{};
    std::string description_{};
    std::string state_{};
    std::string sample_format_{};
    std::string sample_rate_{};
    std::string channels_{};
    std::vector<std::pair<std::string /*port */, std::string /*description */>> ports_{};
    std::string active_port_{};
  } DeviceDescription;

  static const std::string kConnectionSpec;  //!< Shmdata specifications
  std::vector<AudioRingBuffer<float>> ring_buffers_{};  // one per channel
  bool pa_stream_is_on_{false};  // in order to wait for saving audio from shmdata into the ring
                                 // buffers. If not, initial latency may increase
  unsigned long int pa_ts_{0};   // frame timestamp manual computed from pulse audio stream playback
  // time unit, FIXME : assuming same sample rate for both shmdata audio and pulseaudio :(
  DriftObserver<long int> drift_observer_{};
  // resampler for drift correction
  std::unique_ptr<AudioResampler<float>> audio_resampler_{};
  unsigned int debug_buffer_usage_{1000};

  std::unique_ptr<gst::GlibMainLoop> mainloop_;
  // internal use:
  std::string shmpath_{};
  pa_glib_mainloop* pa_glib_mainloop_{nullptr};
  pa_mainloop_api* pa_mainloop_api_{nullptr};
  pa_context* pa_context_{nullptr};
  pa_stream* pa_stream_{nullptr};
  char* server_{nullptr};
  std::vector<DeviceDescription> devices_{};  // indexed by pulse_device_name
  std::mutex devices_mutex_{};
  std::condition_variable devices_cond_{};
  bool connected_to_pulse_{false};
  //  property:
  property::Selection<> devices_enum_{{"none"}, 0};
  property::prop_id_t devices_enum_id_{0};
  // quit
  std::mutex quit_mutex_{};
  std::condition_variable quit_cond_{};
  // Shmdata follower
  shmdata::caps::AudioCaps shmf_caps_{""};
  std::unique_ptr<shmdata::Follower> shmf_{nullptr};

  bool on_shmdata_connect(const std::string& shmdata_path);
  bool on_shmdata_disconnect();
  void on_shmreader_connected(const std::string& shmdata_caps);
  void on_shmreader_disconnected();
  void on_shmreader_data(void* data, size_t data_size);
  bool remake_elements();
  void make_device_description(pa_context* pulse_context);
  void update_output_device();
  static void pa_context_state_callback(pa_context* c, void* userdata);
  static void get_sink_info_callback(pa_context* c,
                                     const pa_sink_info* i,
                                     int is_last,
                                     void* userdata);
  static void on_pa_event_callback(pa_context* c,
                                   pa_subscription_event_type_t t,
                                   uint32_t idx,
                                   void* userdata);
  static gboolean async_get_pulse_devices(void* user_data);
  static gboolean quit_pulse(void* user_data);
  static gboolean async_create_stream(void* user_data);
  static gboolean async_remove_stream(void* user_data);
  static void stream_write_cb(pa_stream* p, size_t nbytes, void* userdata);
};

SWITCHER_DECLARE_PLUGIN(PulseSink);
}  // namespace quiddities
}  // namespace switcher
#endif
