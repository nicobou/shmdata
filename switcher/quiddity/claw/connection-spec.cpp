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

  // check specs for shmdata followers
  connection_spec_->for_each_in_array(".follower", [&](InfoTree* shmspec) {
    // check name and structure
    const auto name = shmspec->branch_get_value("name").as<std::string>();
    auto parsed = check_shmdata_spec(shmspec);
    if (!parsed) {
      msg_ = parsed.msg();
      return;
    }
    // compute an id for this follower and add it to the tree
    auto id = follower_id_generator_.allocate_id();
    if (Ids::kInvalid == id) {
      msg_ = "to many followers, no more ids";
      return;
    }
    shmspec->vgraft("sfid", id);
    follower_ids_.emplace(std::make_pair(id, name));
  });

  // check if errors during shm_spec_fun
  if (!msg_.empty()) return;

  // check if names are unique in follower
  auto follower_names = get_follower_names();
  if (follower_names.end() != std::unique(follower_names.begin(), follower_names.end())) {
    msg_ = "duplicate in shmdata follower names";
    return;
  }

  // check specs for shmdata writers
  connection_spec_->for_each_in_array(".writer", [&](InfoTree* shmspec) {
    // check name and structure
    const auto name = shmspec->branch_get_value("name").as<std::string>();
    auto parsed = check_shmdata_spec(shmspec);
    if (!parsed) {
      msg_ = parsed.msg();
      return;
    }
    // compute an id for this writer and add it to the tree
    auto id = writer_id_generator_.allocate_id();
    if (Ids::kInvalid == id) {
      msg_ = "to many writers, no more ids";
      return;
    }
    shmspec->vgraft("swid", id);
    writer_ids_.emplace(std::make_pair(id, name));
  });

  // check if errors during shm_spec_fun
  if (!msg_.empty()) return;

  // check if names are unique in writer
  auto writer_names = get_writer_names();
  if (writer_names.end() != std::unique(writer_names.begin(), writer_names.end())) {
    msg_ = "duplicate in shmdata writer names";
    return;
  }

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

std::vector<std::string> ConnectionSpec::get_writer_names() const {
  std::vector<std::string> res;
  for (const auto& it : writer_ids_) res.push_back(it.second);
  return res;
}

std::vector<std::string> ConnectionSpec::get_follower_names() const {
  std::vector<std::string> res;
  for (const auto& it : follower_ids_) res.emplace_back(it.second);
  return res;
}

std::string ConnectionSpec::get_follower_name(sfid_t sid) const {
  const auto& it = follower_ids_.find(sid);
  if (follower_ids_.cend() == it) return "";
  return it->second;
}

std::string ConnectionSpec::get_writer_name(swid_t sid) const {
  const auto& it = writer_ids_.find(sid);
  if (writer_ids_.cend() == it) return "";
  return it->second;
}

sfid_t ConnectionSpec::get_sfid(const std::string& name) const {
  for (const auto& it : follower_ids_) {
    if (it.second == name) return it.first;
  }
  return Ids::kInvalid;
}

swid_t ConnectionSpec::get_swid(const std::string& name) const {
  for (const auto& it : writer_ids_) {
    if (it.second == name) return it.first;
  }
  return Ids::kInvalid;
}

bool ConnectionSpec::is_allocated(sfid_t sfid) const {
  return follower_ids_.cend() != follower_ids_.find(sfid);
}

sfid_t ConnectionSpec::get_actual_sfid(sfid_t sfid) {
  auto id = sfid;

  // check if sfid is allocated
  const auto& it = follower_ids_.find(sfid);
  if (follower_ids_.cend() == it) return Ids::kInvalid;

  // check if local sfid is meta and allocate a new one if needed
  if (std::string::npos != it->second.find('%')) {
    // this is a meta shmdata follower, so a new specific sfid is required
    id = follower_id_generator_.allocate_id();
    if (Ids::kInvalid == id) {
      return Ids::kInvalid;
    }
    follower_ids_.emplace(
        std::make_pair(id, stringutils::replace_char(it->second, '%', std::to_string(id))));
  }
  return id;
}

bool ConnectionSpec::release(sfid_t sfid) {
  return false;
}

bool ConnectionSpec::is_actual_writer(swid_t swid) const {
  // check if id is allocated
  const auto& it = writer_ids_.find(swid);
  if (writer_ids_.cend() == it) {
    return false;
  }
  // check if this is an actual or a meta shmdata and fail
  if (std::string::npos != it->second.find('%')) {
    return false;
  }
  return true;
}

InfoTree::ptr ConnectionSpec::get_tree() { return connection_spec_; }

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
