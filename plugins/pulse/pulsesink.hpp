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

/**
 * Pulsesink quiddity. It reads an audio Shmdata, and play it through pulseaudio.
 **/
class PulseSink : public Quiddity {
 public:
  /**
   * Construct a PulseSink object.
   */
  PulseSink(quiddity::Config&&);
  /**
   * Destruct a PulseSink object.
   */
  ~PulseSink();

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

  static const std::string kConnectionSpec;  //!< Shmdata specifications.
  std::vector<utils::AudioRingBuffer<float>>
      ring_buffers_{};           //!< One ring buffer per audio channel.
  bool pa_stream_is_on_{false};  //!< Track the pulse audio stream in order to wait for storing
                                 // Shmdata audio into the ring
                                 //!< buffers. If not, initial latency may be high.
  unsigned long int pa_ts_{
      0};  //!< Frame timestamp, manually computed from pulse audio stream playback.
  utils::DriftObserver<long int>
      drift_observer_{};  //!< Observe audio drift in order to apply correction.
  std::unique_ptr<utils::AudioResampler<float>>
      audio_resampler_{};  //!< Resampler for drift correction.
  unsigned int debug_buffer_usage_{
      1000};  //!< Output log message about buffer usage each debug_buffer_usage_ buffers

  std::unique_ptr<gst::GlibMainLoop> mainloop_;  //!< A mainloop for calls to pulse audio functions.
  std::string shmpath_{};                        //!< Shmdata path to read.
  pa_glib_mainloop* pa_glib_mainloop_{nullptr};  //!< Pulse audio mainloop.
  pa_mainloop_api* pa_mainloop_api_{nullptr};    //!< Pulse audio mainloop API.
  pa_context* pa_context_{nullptr};              //!< Pulse audio context.
  pa_stream* pa_stream_{nullptr};                //!< Playback stream.
  char* server_{nullptr};                        //!< Pulse audio server.
  std::vector<DeviceDescription>
      devices_{};  //!< List of audio devices, indexed by pulse_device_name.
  std::mutex devices_mutex_{};
  std::condition_variable devices_cond_{};
  bool connected_to_pulse_{false};
  property::Selection<> devices_enum_{{"none"},
                                      0};   //!< Quiddity property for audio device selection.
  property::prop_id_t devices_enum_id_{0};  //!< Id of the audio device selection property.
  std::mutex quit_mutex_{};
  std::condition_variable quit_cond_{};
  shmdata::caps::AudioCaps shmf_caps_{""};            //!< Audio caps of the Shmdata.
  std::unique_ptr<shmdata::Follower> shmf_{nullptr};  //!< Shmdata follower.

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
