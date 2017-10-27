/*
 * This file is part of switcher-jack.
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

#ifndef __SWITCHER_JACK_TO_SHMDATA_H__
#define __SWITCHER_JACK_TO_SHMDATA_H__

#include <memory>
#include <mutex>
#include "./jack-client.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class JackToShmdata : public Quiddity, public StartableQuiddity {
 public:
  JackToShmdata(QuiddityConfiguration&&);
  ~JackToShmdata() = default;
  JackToShmdata(const JackToShmdata&) = delete;
  JackToShmdata& operator=(const JackToShmdata&) = delete;

 private:
  unsigned int kMaxNumberOfChannels{128};
  unsigned int num_channels_{1};
  PContainer::prop_id_t num_channels_id_{0};
  std::string client_name_{};
  PContainer::prop_id_t client_name_id_{0};
  bool auto_connect_{true};
  PContainer::prop_id_t auto_connect_id_{0};
  std::string connect_to_{"system:capture_"};
  PContainer::prop_id_t connect_to_id_{0};
  unsigned int index_{1};
  PContainer::prop_id_t index_id_{0};
  std::mutex input_ports_mutex_{};
  std::vector<jack_sample_t> buf_{};
  std::vector<std::string> ports_to_connect_{};
  std::mutex port_to_connect_in_jack_process_mutex_{};
  std::vector<std::pair<std::string, std::string>> port_to_connect_in_jack_process_{};
  std::unique_ptr<ShmdataWriter> shm_{nullptr};
  JackClient jack_client_;
  std::vector<JackPort> input_ports_{};

  bool start() final;
  bool stop() final;
  void update_port_to_connect();
  void connect_ports();
  void on_port(jack_port_t* port);
  static int jack_process(jack_nframes_t nframes, void* arg);
  void on_xrun(uint num_of_missed_samples);
};

SWITCHER_DECLARE_PLUGIN(JackToShmdata);

}  // namespace switcher
#endif
