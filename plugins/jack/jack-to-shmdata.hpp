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
#include "switcher/shmdata-writer.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"
#include "./jack-client.hpp"

namespace switcher {
class JackToShmdata: public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(JackToShmdata);
  JackToShmdata(const std::string &);
  ~JackToShmdata() = default;
  JackToShmdata(const JackToShmdata &) = delete;
  JackToShmdata &operator=(const JackToShmdata &) = delete;

 private:
  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *num_channels_spec_{nullptr};
  unsigned int num_channels_{1};
  GParamSpec *client_name_spec_{nullptr};
  std::string client_name_{};
  GParamSpec *connect_to_spec_{nullptr};
  std::string connect_to_{"system:capture_"};
  GParamSpec *index_spec_{nullptr};
  unsigned int index_{1};
  std::unique_ptr<ShmdataWriter> shm_{nullptr};
  std::mutex input_ports_mutex_{};
  JackClient jack_client_;
  std::vector<JackPort> input_ports_{};
  std::vector<jack_sample_t> buf_{};
  std::vector<std::string> ports_to_connect_{};
  std::mutex  port_to_connect_in_jack_process_mutex_{};
  std::vector<std::pair<std::string, std::string>> port_to_connect_in_jack_process_{};
  
  bool init() final;
  bool start() final;
  bool stop() final;
  void update_port_to_connect();
  void connect_ports();
  void on_port(jack_port_t *port);
  static void set_num_channels(const gint value, void *user_data);
  static gint get_num_channels(void *user_data);
  static void set_client_name(const gchar *value, void *user_data);
  static const gchar *get_client_name(void *user_data);
  static void set_connect_to(const gchar *value, void *user_data);
  static const gchar *get_connect_to(void *user_data);
  static void set_index(const gint value, void *user_data);
  static gint get_index(void *user_data);
  static int jack_process (jack_nframes_t nframes, void *arg);
  void on_xrun(uint num_of_missed_samples);
};

SWITCHER_DECLARE_PLUGIN(JackToShmdata);

}  // namespace switcher
#endif
