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
#include "./audio-ring-buffer.hpp"
#include "./drift-observer.hpp"
#include "./jack-client.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/shmdata-connector.hpp"

namespace switcher {
class ShmdataToJack : public Quiddity {
 public:
  ShmdataToJack(QuiddityConfiguration&&);
  ~ShmdataToJack() = default;
  ShmdataToJack(const ShmdataToJack&) = delete;
  ShmdataToJack& operator=(const ShmdataToJack&) = delete;

 private:
  unsigned int kMaxNumberOfChannels{128};
  bool is_constructed_{false};
  // internal use:
  std::string shmpath_{};
  GstElement* shmdatasrc_{nullptr};
  GstElement* audiobin_{nullptr};
  GstElement* fakesink_{nullptr};
  gulong handoff_handler_{0};
  unsigned short channels_{0};
  unsigned int debug_buffer_usage_{1000};
  std::mutex output_ports_mutex_{};
  std::mutex ports_to_connect_mutex_{};
  std::vector<AudioRingBuffer<jack_sample_t>> ring_buffers_{};  // one per channel
  // jack sample is the time unit, assuming gst pipeline has the same sample
  // rate:
  DriftObserver<jack_nframes_t> drift_observer_{};
  // jack client
  JackClient jack_client_;
  // ports
  std::vector<std::string> ports_to_connect_{};
  std::mutex port_to_connect_in_jack_process_mutex_{};
  std::vector<std::pair<std::string, std::string>> port_to_connect_in_jack_process_{};
  std::vector<JackPort> output_ports_{};
  // properties
  bool auto_connect_{true};
  std::string connect_to_{"system:playback_"};
  PContainer::prop_id_t connect_to_id_{0};
  unsigned int index_{1};
  PContainer::prop_id_t index_id_{0};
  PContainer::prop_id_t auto_connect_id_{0};
  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  // gst pipeline:
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  // shmsubscriber (publishing to the information-tree):
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};

  bool start();
  bool stop();
  void update_port_to_connect();
  void connect_ports();
  void disconnect_ports();
  void on_port(jack_port_t* port);
  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_sochet_path);
  bool can_sink_caps(const std::string& caps);
  bool make_elements();
  void check_output_ports(unsigned int channels);
  void on_xrun(uint num_of_missed_samples);
  static void on_handoff_cb(GstElement* object, GstBuffer* buf, GstPad* pad, gpointer user_data);
  static int jack_process(jack_nframes_t nframes, void* arg);
};

SWITCHER_DECLARE_PLUGIN(ShmdataToJack);
}  // namespace switcher
#endif
