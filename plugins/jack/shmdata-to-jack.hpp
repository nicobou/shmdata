/*
 * This file is part of switcher-jack.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_SHMDATA_TO_JACK_H__
#define __SWITCHER_SHMDATA_TO_JACK_H__

#include <memory>
#include <mutex>

#include "./jack-client.hpp"
#include "switcher/gst/pipeliner.hpp"
#include "switcher/shmdata/gst-tree-updater.hpp"
#include "switcher/utils/audio-resampler.hpp"
#include "switcher/utils/audio-ring-buffer.hpp"
#include "switcher/utils/drift-observer.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;

/**
 * ShmdataToJack quiddity. It reads an audio Shmdata, and play it through Jack.
 **/
class ShmdataToJack : public Quiddity {
 public:
  /**
   * Construct a ShmdataToJack object.
   */
  ShmdataToJack(quiddity::Config&&);
  ~ShmdataToJack() = default;

 private:
  static const std::string kConnectionSpec;  //!< Shmdata specifications.
  unsigned int kMaxNumberOfChannels{128};    //!< Max number of channels handled
  bool is_constructed_{false};

  // internal use:
  std::string shmpath_{};
  GstElement* shmdatasrc_{nullptr};  //!< Shmdatasrc GStreamer element to configure.
  GstElement* audiobin_{
      nullptr};  //!< GStreamer bin for reading and converting samples from the Shmdata.
  gulong handoff_handler_{0};
  unsigned short channels_{0};
  unsigned int debug_buffer_usage_{
      1000};  //!< Output log message about buffer usage each debug_buffer_usage_ buffers
  std::mutex output_ports_mutex_{};
  std::mutex ports_to_connect_mutex_{};
  std::vector<utils::AudioRingBuffer<jack_sample_t>>
      ring_buffers_{};  //!< Ring buffer for audio drift correction. There is one ring buffer per
                        //!< channel.
  utils::DriftObserver<jack_nframes_t>
      drift_observer_{};  //!< Track jack timing in order to correct audio drift.
  std::unique_ptr<utils::AudioResampler<jack_sample_t>>
      audio_resampler_{};  //!< Resample for audio drift correction.

  // Quiddity properties
  std::string client_name_{};
  std::string server_name_{};
  property::prop_id_t client_name_id_{0};
  property::prop_id_t server_name_id_{0};
  bool auto_connect_{true};
  std::string connect_to_{"system:playback_%d"};
  property::prop_id_t connect_to_id_;
  unsigned int index_{1};
  property::prop_id_t index_id_{0};
  property::prop_id_t auto_connect_id_;
  bool connect_all_to_first_{false};
  property::prop_id_t connect_all_to_first_id_;
  bool connect_only_first_{false};
  property::prop_id_t connect_only_first_id_;
  bool do_format_conversion_{true};
  property::prop_id_t do_format_conversion_id_;
  bool do_rate_conversion_{true};
  property::prop_id_t do_rate_conversion_id_;

  std::unique_ptr<gst::Pipeliner> gst_pipeline_;  //!< gst pipeline:

  std::unique_ptr<shmdata::GstTreeUpdater> shm_sub_{
      nullptr};  //!< shmsubscriber (publishing to the information-tree)

  // jack client
  std::vector<std::string> ports_to_connect_{};
  std::mutex port_to_connect_in_jack_process_mutex_{};
  std::vector<std::pair<std::string, std::string>> port_to_connect_in_jack_process_{};
  std::unique_ptr<JackClient> jack_client_{nullptr};
  std::vector<JackPort> output_ports_{};

  bool start();
  bool stop();
  void update_ports_to_connect();
  void connect_ports();
  void disconnect_ports();
  void on_port(jack_port_t* port);
  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_socket_path);
  bool make_elements();
  void on_channel_update(unsigned int channels);
  void clean_output_ports();
  void on_xrun(uint num_of_missed_samples);
  static void on_handoff_cb(GstElement* object, GstBuffer* buf, GstPad* pad, gpointer user_data);
  static int jack_process(jack_nframes_t nframes, void* arg);
};

SWITCHER_DECLARE_PLUGIN(ShmdataToJack);
}  // namespace quiddities
}  // namespace switcher
#endif
