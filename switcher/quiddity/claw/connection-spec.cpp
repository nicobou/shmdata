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
    // check label and structure
    const auto label = shmspec->branch_get_value("label").as<std::string>();
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
    shmspec->vgraft("from_sfid", Ids::kInvalid);
    follower_ids_.emplace(std::make_pair(id, label));
  });

  // check if errors during shm_spec_fun
  if (!msg_.empty()) return;

  // check if labels are unique in follower
  auto follower_labels = get_follower_labels();
  if (follower_labels.end() != std::unique(follower_labels.begin(), follower_labels.end())) {
    msg_ = "duplicate in shmdata follower labels";
    return;
  }

  // check specs for shmdata writers
  connection_spec_->for_each_in_array(".writer", [&](InfoTree* shmspec) {
    // check label and structure
    const auto label = shmspec->branch_get_value("label").as<std::string>();
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
    shmspec->vgraft("from_swid", Ids::kInvalid);
    writer_ids_.emplace(std::make_pair(id, label));
  });

  // check if errors during shm_spec_fun
  if (!msg_.empty()) return;

  // check if labels are unique in writer
  auto writer_labels = get_writer_labels();
  if (writer_labels.end() != std::unique(writer_labels.begin(), writer_labels.end())) {
    msg_ = "duplicate in shmdata writer labels";
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
  const auto label = tree->branch_get_value("label");
  if (!label.is<std::string>())
    return BoolLog(false, "label must be a string: " + infotree::json::serialize(tree));
  if (label.as<std::string>().empty()) return BoolLog(false, "label must not be empty");
  const auto description = tree->branch_get_value("description");
  if (!description.is<std::string>())
    return BoolLog(false, "description must be a string: " + infotree::json::serialize(tree));

  if (!tree->branch_is_array("can_do"))
    return BoolLog(false,
                   "can_do entry must be an array of caps in " + infotree::json::serialize(tree));
  auto can_do_spec_is_correct = true;
  std::string caps_message;
  tree->cfor_each_in_array("can_do", [&](const InfoTree* caps) {
    auto val = caps->get_value();
    if (!val.is<std::string>()) {
      caps_message = std::string("caps must be string value: ") + infotree::json::serialize(caps);
      can_do_spec_is_correct = false;
    }
    auto type = shmdata::Type(val.as<std::string>());
    if (!type.get_parsing_errors().empty()) {
      caps_message = std::string("caps parsing error: ") + type.get_parsing_errors() + "(" +
                     val.as<std::string>() + ")";
      can_do_spec_is_correct = false;
    }
  });
  if (!can_do_spec_is_correct) return BoolLog(false, caps_message);
  return BoolLog(true);
}

std::vector<std::string> ConnectionSpec::get_writer_labels() const {
  std::vector<std::string> res;
  for (const auto& it : writer_ids_) res.push_back(it.second);
  return res;
}

std::vector<std::string> ConnectionSpec::get_follower_labels() const {
  std::vector<std::string> res;
  for (const auto& it : follower_ids_) res.emplace_back(it.second);
  return res;
}

std::string ConnectionSpec::get_follower_label(sfid_t sid) const {
  const auto& it = follower_ids_.find(sid);
  if (follower_ids_.cend() == it) return "";
  return it->second;
}

std::string ConnectionSpec::get_writer_label(swid_t sid) const {
  const auto& it = writer_ids_.find(sid);
  if (writer_ids_.cend() == it) return "";
  return it->second;
}

sfid_t ConnectionSpec::get_sfid(const std::string& label) const {
  for (const auto& it : follower_ids_) {
    if (it.second == label) return it.first;
  }
  return Ids::kInvalid;
}

swid_t ConnectionSpec::get_swid(const std::string& label) const {
  for (const auto& it : writer_ids_) {
    if (it.second == label) return it.first;
  }
  return Ids::kInvalid;
}

bool ConnectionSpec::is_allocated_follower(sfid_t sfid) const {
  return follower_ids_.cend() != follower_ids_.find(sfid);
}

bool ConnectionSpec::is_meta_follower(sfid_t sfid) const {
  const auto& it = follower_ids_.find(sfid);
  if (writer_ids_.cend() == it) {
    return false;
  }
  if (std::string::npos == it->second.find('%')) {
    return false;
  }
  return true;
}

sfid_t ConnectionSpec::allocate_sfid_from_meta(sfid_t sfid) const {
  // check if sfid is allocated
  const auto& it = follower_ids_.find(sfid);
  if (follower_ids_.cend() == it) {
    return Ids::kInvalid;
  }

  // check if local sfid is meta
  if (std::string::npos == it->second.find('%')) {
    return Ids::kInvalid;
  }

  // allocate a new sfid
  auto id = follower_id_generator_.allocate_id();
  if (Ids::kInvalid == id) {
    return Ids::kInvalid;
  }
  auto label = stringutils::replace_char(it->second, '%', std::to_string(id));
  follower_ids_.emplace(std::make_pair(id, label));

  // add to the tree
  InfoTree::ptr follower_tree = InfoTree::make();
  follower_tree->vgraft("label", label);
  follower_tree->vgraft("sfid", id);
  follower_tree->vgraft("from_sfid", sfid);
  connection_spec_->graft("follower." + std::to_string(id), follower_tree);
  return id;
}

bool ConnectionSpec::deallocate_sfid_from_meta(sfid_t sfid) const {
  // check sfid is a dynamic
  if (Ids::kInvalid ==
      connection_spec_->branch_get_value("follower." + std::to_string(sfid) + ".from_sfid")
          .as<sfid_t>()) {
    return false;
  }
  connection_spec_->prune("follower." + std::to_string(sfid));
  follower_ids_.erase(follower_ids_.find(sfid));
  return true;
}

bool ConnectionSpec::is_allocated_writer(swid_t swid) const {
  const auto& it = writer_ids_.find(swid);
  if (writer_ids_.cend() == it) {
    return false;
  }
  return true;
}

bool ConnectionSpec::is_meta_writer(swid_t swid) const {
  const auto& it = writer_ids_.find(swid);
  if (writer_ids_.cend() == it) {
    return false;
  }
  if (std::string::npos != it->second.find('%')) {
    return false;
  }
  return true;
}

swid_t ConnectionSpec::allocate_swid_from_meta(swid_t swid, const shm_spec_t& spec) {
  // check if swid is allocated
  const auto& it = writer_ids_.find(swid);
  if (writer_ids_.cend() == it) {
    return Ids::kInvalid;
  }

  // check if local swid is meta
  if (std::string::npos == it->second.find('%')) {
    return Ids::kInvalid;
  }

  // allocate a new swid
  auto id = writer_id_generator_.allocate_id();
  if (Ids::kInvalid == id) {
    return Ids::kInvalid;
  }
  auto label = spec.label;
  if (label.empty()) {
    label = stringutils::replace_char(it->second, '%', std::to_string(id));
  }
  writer_ids_.emplace(std::make_pair(id, label));

  // add to the tree
  InfoTree::ptr writer_tree = InfoTree::make();
  writer_tree->vgraft("label", label);
  writer_tree->vgraft("swid", id);
  writer_tree->vgraft("description", spec.label);
  writer_tree->vgraft("from_swid", swid);
  connection_spec_->graft("writer." + std::to_string(id), writer_tree);
  return id;
}

bool ConnectionSpec::deallocate_swid_from_meta(swid_t swid) {
  // check if swid is allocated
  const auto& it = writer_ids_.find(swid);
  if (writer_ids_.cend() == it) {
    return false;
  }

  // check if swid is genreated from an other swid
  if (Ids::kInvalid ==
      connection_spec_->branch_get_value("writer." + std::to_string(swid) + ".from_swid")
          .as<swid_t>()) {
    return false;
  }

  // deallocate the swid
  writer_ids_.erase(writer_ids_.find(swid));

  // remove from the tree
  connection_spec_->prune("writer." + std::to_string(swid));
  return swid;
}

InfoTree::ptr ConnectionSpec::get_tree() { return connection_spec_; }

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
