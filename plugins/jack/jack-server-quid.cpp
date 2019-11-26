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

#include "./jack-server-quid.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackServerQuid,
                                     "jackserver",
                                     "Jack Audio Server",
                                     "audio",
                                     "",
                                     "Run a Jack audio server",
                                     "LGPL",
                                     "Nicolas Bouillot");

JackServerQuid::JackServerQuid(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      StartableQuiddity(this),
      jack_server_(get_log_ptr(),
                   config<MPtr(&InfoTree::branch_get_value)>("name"),
                   config<MPtr(&InfoTree::branch_get_value)>("driver"),
                   config<MPtr(&InfoTree::branch_get_value)>("realtime").is_null()
                       ? false
                       : config<MPtr(&InfoTree::branch_get_value)>("realtime").as<bool>()),
      config_(jack_server_.get_config()),
      driver_config_id_(pmanage<MPtr(&PContainer::make_group)>(
          "driver_config",
          "Driver configuration",
          "Select if you want to configure the audio driver used by jack.")),
      advanced_config_id_(pmanage<MPtr(&PContainer::make_group)>(
          "advanced_config",
          "Advanced configuration",
          "Select if you want to configure jack's advanced configuration.")),
      driver_enum_(jack_server_.get_driver_names(), jack_server_.get_driver_index()),
      driver_id_(pmanage<MPtr(&PContainer::make_parented_selection<>)>(
          "driver",
          "driver_config",
          [this](const IndexOrName& val) {
            driver_enum_.select(val);
            if (!jack_server_.update_driver(driver_enum_.get_current())) return false;
            renew_driver_properties();
            return true;
          },
          [this]() { return driver_enum_.get(); },
          "Audio drivers",
          "Audio driver to be used by the Jack server.",
          driver_enum_)) {
  if (!jack_server_) {
    is_valid_ = false;
    return;
  }
  // creating Jack related properties
  auto params = config_->get_child_keys("params");
  for (const auto& it : params) make_param("params." + it, "advanced_config");

  // creating driver properties
  renew_driver_properties();
}

void JackServerQuid::renew_driver_properties() {
  // removing old properties
  for (const auto& it : driver_params_) pmanage<MPtr(&PContainer::remove)>(it);

  // creating new driver related properties
  auto params = config_->get_child_keys("driver.params");
  for (const auto& it : params) {
    auto id = make_param("driver.params." + it, "driver_config");
    if (id != 0) driver_params_.push_back(id);
  }
}

bool JackServerQuid::start() { return jack_server_.start(); }

bool JackServerQuid::stop() { return jack_server_.stop(); }

PContainer::prop_id_t JackServerQuid::make_param(const std::string& config_path,
                                                 const std::string& parent) {
  auto type = config_->branch_get_value(config_path + ".type").as<std::string>();
  auto name = config_->branch_get_value(config_path + ".name").as<std::string>();
  if (type == "bool") {
    return pmanage<MPtr(&PContainer::make_parented_bool)>(
        name,
        parent,
        [this, config_path](const bool& val) {
          return config_->branch_set_value(config_path + ".value", val);
        },
        [this, config_path]() {
          return config_->branch_get_value(config_path + ".value").as<bool>();
        },
        config_->branch_get_value(config_path + ".short description").as<std::string>(),
        config_->branch_get_value(config_path + ".long description").as<std::string>(),
        config_->branch_get_value(config_path + ".value").as<bool>());
  }
  if (type == "string") {
    return pmanage<MPtr(&PContainer::make_parented_string)>(
        name,
        parent,
        [this, config_path](const std::string& val) {
          return config_->branch_set_value(config_path + ".value", val);
        },
        [this, config_path]() {
          return config_->branch_get_value(config_path + ".value").as<std::string>();
        },
        config_->branch_get_value(config_path + ".short description").as<std::string>(),
        config_->branch_get_value(config_path + ".long description").as<std::string>(),
        config_->branch_get_value(config_path + ".value").as<std::string>());
  }
  if (type == "int") {
    auto min = std::numeric_limits<int>::min();
    auto max = std::numeric_limits<int>::max();
    auto config_min = config_->branch_get_value(config_path + ".min");
    if (config_min.not_null()) min = config_min;
    auto config_max = config_->branch_get_value(config_path + ".max");
    if (config_max.not_null()) max = config_max;
    return pmanage<MPtr(&PContainer::make_parented_int)>(
        name,
        parent,
        [this, config_path](const int& val) {
          return config_->branch_set_value(config_path + ".value", val);
        },
        [this, config_path]() {
          return config_->branch_get_value(config_path + ".value").as<int>();
        },
        config_->branch_get_value(config_path + ".short description").as<std::string>(),
        config_->branch_get_value(config_path + ".long description").as<std::string>(),
        config_->branch_get_value(config_path + ".value").as<int>(),
        min,
        max);
  }
  if (type == "uint") {
    auto min = std::numeric_limits<uint>::min();
    auto max = std::numeric_limits<uint>::max();
    auto config_min = config_->branch_get_value(config_path + ".min");
    if (config_min.not_null()) min = config_min;
    auto config_max = config_->branch_get_value(config_path + ".max");
    if (config_max.not_null()) max = config_max;
    return pmanage<MPtr(&PContainer::make_parented_unsigned_int)>(
        name,
        parent,
        [this, config_path](const uint& val) {
          return config_->branch_set_value(config_path + ".value", val);
        },
        [this, config_path]() {
          return config_->branch_get_value(config_path + ".value").as<uint>();
        },
        config_->branch_get_value(config_path + ".short description").as<std::string>(),
        config_->branch_get_value(config_path + ".long description").as<std::string>(),
        config_->branch_get_value(config_path + ".value").as<uint>(),
        min,
        max);
  }
  if (type == "enum") {
    // prepare values and description for selection
    std::vector<std::string> values;
    config_->copy_and_insert_child_keys(
        config_path + ".enum", std::insert_iterator<decltype(values)>(values, values.begin()));
    std::vector<std::string> value_names;
    for (const auto& it : config_->copy_leaf_values(config_path + ".enum"))
      value_names.push_back(it);
    selections_.emplace(config_path, Selection<>(std::move(value_names), std::move(values), 0));
    selections_.at(config_path)
        .select(config_->branch_get_value(config_path + ".value").as<std::string>());
    // return created property
    return pmanage<MPtr(&PContainer::make_parented_selection<>)>(
        name,
        parent,
        [this, config_path](const IndexOrName& val) {
          auto selection = selections_.at(config_path);
          selection.select(val);
          config_->branch_set_value(config_path + ".value", selection.get_current_index());
          return true;
        },
        [this, config_path]() { return selections_.at(config_path).get(); },
        config_->branch_get_value(config_path + ".short description").as<std::string>(),
        config_->branch_get_value(config_path + ".long description").as<std::string>(),
        selections_.at(config_path));
  }
  if (type == "char") {
    return pmanage<MPtr(&PContainer::make_parented_char)>(
        name,
        parent,
        [this, config_path](const char& val) {
          return config_->branch_set_value(config_path + ".value", val);
        },
        [this, config_path]() {
          return config_->branch_get_value(config_path + ".value").as<char>();
        },
        config_->branch_get_value(config_path + ".short description").as<std::string>(),
        config_->branch_get_value(config_path + ".long description").as<std::string>(),
        config_->branch_get_value(config_path + ".value").as<char>());
  }
  warning("jackserver: unknown type (%) for Jack parameter named %", type, config_path);
  return 0;
}

}  // namespace switcher
