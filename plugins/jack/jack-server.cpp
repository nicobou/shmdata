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

#include "./jack-server.hpp"
#include <cstring>

namespace switcher {

log::BaseLogger* JackServer::current_jack_log_ = nullptr;

JackServer::JackServer(log::BaseLogger* logger,
                       const std::string& name,
                       const std::string& config_driver,
                       bool realtime)
    : log_(logger),
      server_(jackctl_server_create(nullptr, nullptr)),
      config_(InfoTree::make()),
      default_driver_(config_driver.empty() ? kDefaultDriver : config_driver) {
  {
    current_jack_log_ = log_;
    // routing jack log into switcher log:
    jack_set_info_function(JackServer::jack_info);
    jack_set_error_function(JackServer::jack_error);

    // parameter introspection
    if (!server_) {
      log_->error("jack-server initialization failed");
      return;
    }
    InfoTree::ptr params = InfoTree::make();
    auto it = jackctl_server_get_parameters(server_);
    while (it) {
      // (using parameter name as ID because the jackctl_parameter_get_id returns an empty char)
      auto param = static_cast<jackctl_parameter_t*>(it->data);
      std::string id(jackctl_parameter_get_name(param));
      write_parameter_to_tree(params, id, param);
      it = jack_slist_next(it);
    }
    params->branch_set_value(".name.value", name);
    params->branch_set_value(".realtime.value", realtime);
    log_->debug("jack-server-quid: initial configuration%", params->serialize_json("."));
    // adding to the config
    config_->graft(".params", params);
  }

  // Drivers list
  // current_driver will be set either to default or (if not available) to the first one in the list
  std::string current_driver;
  {
    auto it = jackctl_server_get_drivers_list(server_);
    while (it) {
      auto itdriver = static_cast<jackctl_driver_t*>(it->data);
      auto name = std::string(jackctl_driver_get_name(itdriver));
      // selecting default driver:
      if (name == default_driver_ || current_driver.empty()) current_driver = name;
      // master or slave
      if (jackctl_driver_get_type(itdriver) == JackMaster) {
        drivers_.push_back(name);
      } else {
        slave_drivers_.push_back(name);
      }
      it = jack_slist_next(it);
    }
    if (drivers_.empty()) {
      log_->warning("BUG: no driver available for jack server");
      return;
    }
    config_->graft(".driver.name", InfoTree::make(current_driver));
  }

  // current driver params
  log_->debug("jack server: % is set as default driver", current_driver);
  update_driver(current_driver);
}

bool JackServer::update_driver(const std::string& driver_name) {
  auto driver = get_driver(driver_name);
  if (!driver) return false;
  current_driver_ = driver;

  // maintaining current_driver_index_
  current_driver_index_ = 0;
  for (const auto& it : drivers_) {
    if (driver_name == it) break;
    ++current_driver_index_;
  }

  // updating config_
  config_->vgraft("driver.name", driver_name);
  InfoTree::ptr params = InfoTree::make();
  auto it = jackctl_driver_get_parameters(current_driver_);
  while (it) {
    // (using parameter name as ID because the jackctl_parameter_get_id returns an empty char)
    auto param = static_cast<jackctl_parameter_t*>(it->data);
    std::string id(jackctl_parameter_get_name(param));
    write_parameter_to_tree(params, id, param);
    it = jack_slist_next(it);
  }
  // adding to the config
  config_->graft("driver.params", params);

  return true;
}

JackServer::~JackServer() {
  jackctl_server_destroy(server_);
  if (current_jack_log_ == log_) current_jack_log_ = nullptr;
}

jackctl_driver_t* JackServer::get_driver(const std::string& driver_name) {
  const JSList* node_ptr = jackctl_server_get_drivers_list(server_);
  while (node_ptr) {
    auto driver = static_cast<jackctl_driver_t*>(node_ptr->data);
    if (std::string(jackctl_driver_get_name(driver)) == driver_name) {
      return driver;
    }
    node_ptr = jack_slist_next(node_ptr);
  }
  return nullptr;
}

bool JackServer::safe_bool_idiom() const {
  if (!server_) return false;
  return true;
}

void JackServer::write_parameter_to_tree(const InfoTree::ptr& tree,
                                         const std::string& key,
                                         jackctl_parameter_t* param) {
  tree->vgraft(key + ".name", jackctl_parameter_get_name(param));
  tree->vgraft(key + ".short description", jackctl_parameter_get_short_description(param));
  tree->vgraft(key + ".long description", jackctl_parameter_get_long_description(param));

  union jackctl_parameter_value val = jackctl_parameter_get_value(param);

  switch (jackctl_parameter_get_type(param)) {
    case JackParamInt:
      tree->vgraft(key + ".value", val.i);
      tree->vgraft(key + ".type", "int");
      break;
    case JackParamUInt:
      tree->vgraft(key + ".value", val.ui);
      tree->vgraft(key + ".type", "uint");
      break;
    case JackParamChar:
      tree->vgraft(key + ".value", val.c);
      tree->vgraft(key + ".type", "char");
      break;
    case JackParamString:
      tree->vgraft(key + ".value", val.str);
      tree->vgraft(key + ".type", "string");
      break;
    case JackParamBool:
      tree->vgraft(key + ".value", val.b);
      tree->vgraft(key + ".type", "bool");
      break;
    default:
      log_->warning("BUG: no type for parameter % in jack server",
                    std::string(jackctl_parameter_get_name(param)));
      break;
  }

  if (jackctl_parameter_has_enum_constraint(param)) {
    // note that jackdmp is using the char value for enum:
    // jackctl_parameter_get_enum_constraint_value(param, i).c
    // but this fail with alsa devices since they all have the same index 'h',
    // so we are using here our own index in order to avoid index collision
    tree->vgraft(key + ".type", "enum");
    for (uint32_t i = 0; i < jackctl_parameter_get_enum_constraints_count(param); ++i) {
      if (val.c == jackctl_parameter_get_enum_constraint_value(param, i).c)
        tree->vgraft(key + ".value", i);
      tree->vgraft(key + ".enum." + std::to_string(i),
                   jackctl_parameter_get_enum_constraint_description(param, i));
    }
  }

  if (jackctl_parameter_has_range_constraint(param)) {
    union jackctl_parameter_value min, max;
    jackctl_parameter_get_range_constraint(param, &min, &max);
    tree->vgraft(key + ".min", min.i);
    tree->vgraft(key + ".max", max.i);
  }
}

bool JackServer::start() {
  log_->debug("jack-server starting: apply config");
  if (!apply_config()) {
    log_->warning("jackserver: failed to apply configuration before starting");
    return false;
  }
  log_->debug("jack-server starting: open server");
  if (!jackctl_server_open(server_, current_driver_)) {
    log_->warning("jackserver: failed to open server before starting");
    return false;
  }

  log_->debug("jack-server starting: start !");
  return jackctl_server_start(server_);
}

bool JackServer::stop() {
  if (!jackctl_server_stop(server_)) return false;
  return jackctl_server_close(server_);
}

bool JackServer::apply_config() {
  {  // server parameters
    auto it = jackctl_server_get_parameters(server_);
    while (it) {
      // (using parameter name as ID because the jackctl_parameter_get_id returns an empty char)
      auto param = static_cast<jackctl_parameter_t*>(it->data);
      if (!apply_param(std::string(".params.") + jackctl_parameter_get_name(param), param)) {
        log_->warning("jack server: failed to apply parameter %",
                      jackctl_parameter_get_name(param));
      }
      it = jack_slist_next(it);
    }
  }

  {  // driver parameters
    auto it = jackctl_driver_get_parameters(current_driver_);
    while (it) {
      auto param = static_cast<jackctl_parameter_t*>(it->data);
      if (!apply_param(std::string("driver.params.") + jackctl_parameter_get_name(param), param))
        return false;
      it = jack_slist_next(it);
    }
  }
  return true;
}

bool JackServer::apply_param(const std::string& key, jackctl_parameter_t* param) {
  union jackctl_parameter_value val;

  if (jackctl_parameter_has_enum_constraint(param)) {
    auto id = Any::to_string(config_->branch_get_value(key + ".value"));
    auto label = config_->branch_get_value(key + ".enum." + id).as<std::string>();
    for (uint32_t i = 0; i < jackctl_parameter_get_enum_constraints_count(param); ++i) {
      if (label == jackctl_parameter_get_enum_constraint_description(param, i)) {
        val = jackctl_parameter_get_enum_constraint_value(param, i);
        jackctl_parameter_set_value(param, &val);
        return true;
      }
    }
    // enum value not found
    log_->warning(
        "jack server configuration: failed to apply enum value % (key: %, id: %)", label, key, id);
    return false;
  }

  std::string strval;
  switch (jackctl_parameter_get_type(param)) {
    case JackParamInt:
      val.i = config_->branch_get_value(key + ".value");
      break;
    case JackParamUInt:
      val.ui = config_->branch_get_value(key + ".value");
      break;
    case JackParamChar:
      val.c = config_->branch_get_value(key + ".value");
      break;
    case JackParamString:
      strval = config_->branch_get_value(key + ".value").as<std::string>();
      if (strval.empty()) {
        log_->debug("jack server: empty string value not applied for %",
                    std::string(jackctl_parameter_get_name(param)));
        return true;
      }
      strncpy(val.str, strval.c_str(), JACK_PARAM_STRING_MAX);
      break;
    case JackParamBool:
      val.b = config_->branch_get_value(key + ".value");
      break;
    default:
      log_->warning("BUG: no type for parameter % in jack server",
                    std::string(jackctl_parameter_get_name(param)));
      break;
  }
  jackctl_parameter_set_value(param, &val);
  return true;
}

unsigned int JackServer::get_driver_index() const { return current_driver_index_; }

void JackServer::jack_info(const char* msg) {
  if (current_jack_log_) current_jack_log_->debug("jackd info: %", std::string(msg));
}

void JackServer::jack_error(const char* msg) {
  if (current_jack_log_) current_jack_log_->error("jackd info: %", std::string(msg));
}

}  // namespace switcher
