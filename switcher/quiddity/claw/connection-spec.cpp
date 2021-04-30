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

#include "./connection-spec.hpp"

#include <shmdata/type.hpp>
#include <string>

#include "../../infotree/json-serializer.hpp"

namespace switcher {
namespace quiddity {
namespace claw {

ConnectionSpec::ConnectionSpec() : BoolLog(true), connection_spec_(InfoTree::make()) {}

ConnectionSpec::ConnectionSpec(const std::string& spec)
    : connection_spec_(infotree::json::deserialize(spec, true)) {
  if (connection_spec_->branch_has_data(".parsing_error")) {
    msg_ = connection_spec_->branch_get_value(".parsing_error").as<std::string>();
    return;
  }
  for (const auto& it : connection_spec_->get_child_keys(".")) {
    if (it != "follower" && it != "writer") {
      msg_ = std::string("unexpected key: ") + it;
      return;
    }
    if (!connection_spec_->branch_is_array(it)) {
      msg_ = std::string("key ") + it + " must be an array";
      return;
    }
  }
  bool shm_spec_is_valid = true;
  std::vector<std::string> names;
  auto shm_spec_fun = [&](const InfoTree* shmspec) {
    const auto name = shmspec->branch_get_value("name").as<std::string>();
    auto parsed = check_shmdata_spec(shmspec);
    if (!parsed) {
      msg_ = parsed.msg();
      return;
    }
    names.emplace_back(name);
  };
  // check specs for shmdata followers
  connection_spec_->cfor_each_in_array(".follower", shm_spec_fun);
  // check if errors during shm_spec_fun
  if (!msg_.empty()) return;
  // check if names are unique in follower
  if (names.end() != std::unique(names.begin(), names.end())) {
    msg_ = "duplicate in shmdata follower names";
    return;
  }
  names.clear();
  // check specs for shmdata writers
  connection_spec_->cfor_each_in_array(".writer", shm_spec_fun);
  // check if errors during shm_spec_fun
  if (!msg_.empty()) return;
  // check if names are unique in writer
  if (names.end() != std::unique(names.begin(), names.end())) {
    msg_ = "duplicate in shmdata writer names";
    return;
  }
  // exit a follower or writer specs are not valid
  if (!shm_spec_is_valid) return;

  // finally all went well
  is_valid_ = true;
}

BoolLog ConnectionSpec::check_shmdata_spec(const InfoTree* tree) {
  const auto keys = tree->get_child_keys(".");
  if (3 != keys.size())
    return BoolLog(false,
                   std::string("unexpected number of key for ") + infotree::json::serialize(tree));
  const auto name = tree->branch_get_value("name");
  if (!name.is<std::string>())
    return BoolLog(false, "name must be a string: " + infotree::json::serialize(tree));
  if (name.as<std::string>().empty()) return BoolLog(false, "name must not be empty");
  const auto description = tree->branch_get_value("description");
  if (!description.is<std::string>())
    return BoolLog(false, "description must be a string: " + infotree::json::serialize(tree));

  if (!tree->branch_is_array("caps"))
    return BoolLog(false, "caps entry must be an array in " + infotree::json::serialize(tree));
  auto caps_spec_is_correct = true;
  std::string caps_message;
  tree->cfor_each_in_array("caps", [&](const InfoTree* caps) {
    auto val = caps->get_value();
    if (!val.is<std::string>()) {
      caps_message = std::string("caps must be string value: ") + infotree::json::serialize(caps);
      caps_spec_is_correct = false;
    }
    auto type = shmdata::Type(val.as<std::string>());
    if (!type.get_parsing_errors().empty()) {
      caps_message = std::string("caps parsing error: ") + type.get_parsing_errors() + "(" +
                     val.as<std::string>() + ")";
      caps_spec_is_correct = false;
    }
  });
  if (!caps_spec_is_correct) return BoolLog(false, caps_message);
  return BoolLog(true);
}

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
