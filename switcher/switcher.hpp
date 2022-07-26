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

#ifndef __SWITCHER_SWITCHER_H__
#define __SWITCHER_SWITCHER_H__

#include <spdlog/sinks/stdout_color_sinks.h>
#include <uuid/uuid.h>

#include <cstdlib>
#include <filesystem>
#include <regex>
#include <string>
#include <vector>

#include "configuration/configuration.hpp"
#include "gst/initialized.hpp"
#include "infotree/information-tree.hpp"
#include "logger/logger.hpp"
#include "quiddity/container.hpp"
#include "quiddity/factory.hpp"
#include "quiddity/quid-id-t.hpp"
#include "session/session.hpp"
#include "utils/make-consultable.hpp"
#include "utils/string-utils.hpp"

namespace fs = std::filesystem;

namespace switcher {

// forward declarations
namespace quiddity::bundle {
class Bundle;
}  // namespace quiddity::bundle
namespace session {
class Session;
}  // namespace session

using namespace session;

class Switcher : public gst::Initialized {
  friend class quiddity::bundle::Bundle;  // access to qcontainer_ and qfactory_
 public:
  /**
   * @brief Universally unique identifier
   * @details A universally unique identifier is a 128-bit label used for information in computer
   * systems.
   */
  const std::string uuid;

  /**
   * @brief The name of the Switcher instance
   * @details A name is given in constructor parameters when `making` a Switcher instance
   */
  const std::string name;

  /**
   * @brief A shared pointer to a registered logger
   * @details When `making` the Switcher instance, a logger is initialized and outputs only to the
   * console until the instance is configured. Once the instance can check for `logs` settings, the
   * logger will also outputs to a log file on the system.
   */
  std::shared_ptr<spdlog::logger> logger;

  /**
   * @brief The session of the Swicher instance
   * @details Every Switcher instance has its own session to manage session files to keep a
   *          history of the current state of a Switcher instance.
   */
  std::shared_ptr<Session> session;

  /**
   * @brief Shared pointer type
   */
  using ptr = std::shared_ptr<Switcher>;

  static Switcher::ptr make_switcher(const std::string& name, bool debug = false);
  Switcher(const Switcher&) = delete;
  ~Switcher() = default;

  Switcher& operator=(const Switcher&) = delete;

  std::string get_switcher_version() const;
  std::string get_switcher_caps() const;

  int get_control_port() const;
  void set_control_port(const int port);

  // State
  InfoTree::ptr get_state() const;
  bool load_state(InfoTree* state);
  void reset_state(bool remove_created_quiddities);

  // Quiddity Factory
  Make_delegate(Switcher, quiddity::Factory, &qfactory_, factory);

  // Quiddity container
  Make_delegate(Switcher, quiddity::Container, qcontainer_.get(), quids);

  // Configuration
  Make_delegate(Switcher, Configuration, &conf_, conf);

  // shmpaths
  const std::string get_shm_dir() const { return conf_.get_value(".shm.directory"); }
  const std::string get_shm_prefix() const { return conf_.get_value(".shm.prefix"); }

  // Bundles
  bool load_bundle_from_config(const std::string& bundle_description);

  bool register_bundle(InfoTree::ptr bundle_tree);
  bool unregister_bundle(InfoTree::ptr bundle_tree);

 private:
  /**
   * @brief Generate a universally unique identifier (uuid)
   * @return A string that contains the generated uuid
   */
  std::string make_uuid();

  /**
   * @brief Initialize the logger and/or add sinks to it
   * @details When `making` a Switcher instance, this initializes the logger
   *          and adds a `console sink` to it. If it already is initialized,
   *          it only push a new `console sink` to the `sinks` vector of
   *          the logger instance.
   *
   * @param debug A boolean that indicates if a `debug` log level should be enforced
   * @return A shared pointer to the logger
   */
  std::shared_ptr<spdlog::logger> make_logger(bool debug = false);

  /**
   * @brief Deletes the default constructor
   */
  Switcher() = delete;

  /**
   * @brief Constructor
   *
   * @param switcher_name The name of the Switcher instance
   * @param debug A boolean that indicates if a `debug` log level should be enforced
   */
  Switcher(const std::string& switcher_name, bool debug = false);

  /**
   * Scan a directory for plugins and load them. The scan is recursive.
   *
   * @param path the path of the directory
   */
  void scan_dir_for_plugins(const std::string& path);

  void remove_shm_zombies() const;
  void apply_gst_configuration();
  void register_bundle_from_configuration();
  static void init_gst();

  quiddity::Factory qfactory_;
  quiddity::Container::ptr qcontainer_;
  Configuration conf_;
  std::vector<quiddity::qid_t> quiddities_at_reset_{};
  std::weak_ptr<Switcher> me_{};
  int control_port_{0};
};
}  // namespace switcher

#endif
