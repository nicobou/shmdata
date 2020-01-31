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

#ifndef __SWITCHER_JACK_SERVER_H__
#define __SWITCHER_JACK_SERVER_H__

#include <jack/control.h>
#include <jack/jack.h>

#include <string>
#include "switcher/infotree/information-tree.hpp"
#include "switcher/logger/base.hpp"
#include "switcher/utils/safe-bool-idiom.hpp"

/* The JackServer class implements the control of a jackserver. Default configuration is obtained
 * in the constructor through introspection provided by the jackd API. This configuration is saved
 * in an InfoTree. This class should be used as follows: construct a JackServerObject, get its
 * shared InfoTree, change the values in the infotree, then start the jackserver. The JackServerQuid
 * class implements this behaviour. Note that changes in the configuration are applied only when
 * starting the Jack server. */

namespace switcher {
namespace quiddities {

class JackServer : public SafeBoolIdiom {
 public:
  explicit JackServer(log::Base* logger,
                      const std::string& name,
                      const std::string& driver,
                      bool real_time);
  JackServer() = delete;
  ~JackServer();

  // access the config and modify it through use of the InfoTree shared pointer
  InfoTree::ptr get_config() { return config_; };

  // get list of available drivers.
  // example of driver names are dummy, alsa, net, netone
  // example slave driver names are alsarawmidi and loopback
  std::vector<std::string> get_driver_names() const { return drivers_; };
  unsigned int get_driver_index() const;
  std::vector<std::string> get_slave_driver_names() const { return slave_drivers_; };

  bool update_driver(const std::string& driver_name);
  // start will apply the config
  bool start();
  bool stop();

 private:
  const std::string kDefaultDriver{"alsa"};
  log::Base* log_;
  jackctl_server_t* server_;
  InfoTree::ptr config_;
  jackctl_driver_t* current_driver_{nullptr};
  unsigned int current_driver_index_{0};
  std::vector<std::string> drivers_{};
  std::vector<std::string> slave_drivers_{};
  std::string default_driver_;

  // unfortunately jack_set_info_function and jack_set_error_function
  // does not provide user data allowing to sentd context when login,
  // so jack log handling is global
  static log::Base* current_jack_log_;

  static void jack_info(const char* msg);
  static void jack_error(const char* msg);
  jackctl_driver_t* get_driver(const std::string& driver_name);
  void write_parameter_to_tree(const InfoTree::ptr& tree,
                               const std::string& key,
                               jackctl_parameter_t* param);
  bool apply_config();
  bool apply_param(const std::string& key, jackctl_parameter_t* param);

  bool safe_bool_idiom() const final;
};

}  // namespace quiddities
}  // namespace switcher
#endif
