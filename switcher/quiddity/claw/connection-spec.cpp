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
    // collect can_do in order to fill follower_can_do_
    std::vector<::shmdata::Type> can_do;
    shmspec->cfor_each_in_array("can_do", [&](const InfoTree* caps) {
      can_do.push_back(::shmdata::Type(caps->get_value().as<std::string>()));
    });
    follower_can_do_.emplace(std::make_pair(id, can_do));
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
    // collect can_do in order to fill writer_can_do_
    std::vector<::shmdata::Type> can_do;
    shmspec->cfor_each_in_array("can_do", [&](const InfoTree* caps) {
      can_do.push_back(::shmdata::Type(caps->get_value().as<std::string>()));
    });
    writer_can_do_.emplace(std::make_pair(id, can_do));
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
  int num_can_do = 0;
  tree->cfor_each_in_array("can_do", [&](const InfoTree* caps) {
    ++num_can_do;
    auto val = caps->get_value();
    if (!val.is<std::string>()) {
      caps_message = std::string("caps must be string value: ") + infotree::json::serialize(caps);
      can_do_spec_is_correct = false;
    }
    auto type = ::shmdata::Type(val.as<std::string>());
    if (!type.get_parsing_errors().empty()) {
      caps_message = std::string("caps parsing error: ") + type.get_parsing_errors() + "(" +
                     val.as<std::string>() + ")";
      can_do_spec_is_correct = false;
    }
  });
  if (0 == num_can_do) {
    return BoolLog(
        false,
        "can_do must contain at least one entry, please use \"all\" is anything is accepted");
  }
  if (!can_do_spec_is_correct) return BoolLog(false, caps_message);
  return BoolLog(true);
}

std::vector<std::string> ConnectionSpec::get_writer_labels() const {
  std::vector<std::string> res;
  for (const auto& writer_id : writer_ids_) res.push_back(writer_id.second);
  return res;
}

std::vector<std::string> ConnectionSpec::get_follower_labels() const {
  std::vector<std::string> res;
  for (const auto& follower_id : follower_ids_) res.emplace_back(follower_id.second);
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
  follower_can_do_.emplace(std::make_pair(id, follower_can_do_[sfid]));
  dynamic_sfids_.emplace_back(id);
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
  auto dynamic_sfid = std::find(dynamic_sfids_.begin(), dynamic_sfids_.end(), sfid);
  if (dynamic_sfids_.end() == dynamic_sfid) {
    return false;
  }
  connection_spec_->prune("follower." + std::to_string(sfid));
  follower_ids_.erase(follower_ids_.find(sfid));
  follower_can_do_.erase(follower_can_do_.find(sfid));
  dynamic_sfids_.erase(dynamic_sfid);
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
  if (std::string::npos == it->second.find('%')) {
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
  writer_can_do_.emplace(std::make_pair(id, writer_can_do_[swid]));
  dynamic_swids_.emplace_back(id);

  // add to the tree
  InfoTree::ptr writer_tree = InfoTree::make();
  writer_tree->vgraft("label", label);
  writer_tree->vgraft("swid", id);
  writer_tree->vgraft("description", spec.description);
  writer_tree->vgraft("from_swid", swid);
  if (spec.can_dos.empty()) {
    const auto& it = writer_can_do_.find(swid);
    int i = 0;
    for (const auto& can_do : it->second) {
      writer_tree->vgraft("can_do." + std::to_string(i), can_do.str());
      ++i;
    }
  } else {
    int i = 0;
    for (const auto& can_do : spec.can_dos) {
      writer_tree->vgraft("can_do." + std::to_string(i), can_do);
      ++i;
    }
  }
  writer_tree->tag_as_array("can_do", true);
  connection_spec_->graft("writer." + std::to_string(id), writer_tree);
  return id;
}

bool ConnectionSpec::deallocate_swid_from_meta(swid_t swid) {
  // check if swid is genreated from an other swid
  auto dynamic_swid = std::find(dynamic_swids_.begin(), dynamic_swids_.end(), swid);
  if (dynamic_swids_.end() == dynamic_swid) {
    return false;
  }

  // deallocate the swid
  writer_ids_.erase(writer_ids_.find(swid));
  writer_can_do_.erase(writer_can_do_.find(swid));
  dynamic_swids_.erase(dynamic_swid);

  // remove from the tree
  connection_spec_->prune("writer." + std::to_string(swid));
  return true;
}

InfoTree::ptr ConnectionSpec::get_tree() { return connection_spec_; }

std::vector<::shmdata::Type> ConnectionSpec::get_follower_can_do(sfid_t sfid) const {
  const auto& it = follower_can_do_.find(sfid);
  if (follower_can_do_.cend() == it) return {};
  return it->second;
}

std::vector<::shmdata::Type> ConnectionSpec::get_writer_can_do(swid_t swid) const {
  const auto& it = writer_can_do_.find(swid);
  if (writer_can_do_.cend() == it) return {};
  return it->second;
}

std::vector<swid_t> ConnectionSpec::get_writer_swids() const {
  std::vector<swid_t> res;
  for (const auto& it : writer_ids_) {
    res.emplace_back(it.first);
  }
  return res;
}

std::vector<sfid_t> ConnectionSpec::get_follower_sfids() const {
  std::vector<sfid_t> res;
  for (const auto& it : follower_ids_) {
    res.emplace_back(it.first);
  }
  return res;
}

bool ConnectionSpec::replace_follower_can_do(sfid_t sfid,
                                             const std::vector<::shmdata::Type>& types) {
  if (types.empty() || !is_allocated_follower(sfid)) {
    return false;
  }
  auto tree = InfoTree::make();
  int i = 0;
  for (const auto& type : types) {
    tree->vgraft(std::to_string(i), type.str());
  }
  connection_spec_->for_each_in_array("follower", [&](InfoTree* element) {
    if (element->branch_get_value("sfid").as<decltype(sfid)>() == sfid) {
      element->graft("can_do", tree);
      element->tag_as_array("can_do", true);
    }
  });
  follower_can_do_[sfid] = types;
  return true;
}

bool ConnectionSpec::replace_writer_can_do(swid_t swid, const std::vector<::shmdata::Type>& types) {
  if (types.empty() || !is_allocated_writer(swid)) {
    return false;
  }
  auto tree = InfoTree::make();
  int i = 0;
  for (const auto& type : types) {
    tree->vgraft(std::to_string(i), type.str());
  }
  connection_spec_->for_each_in_array("writer", [&](InfoTree* element) {
    if (element->branch_get_value("swid").as<decltype(swid)>() == swid) {
      element->graft("can_do", tree);
      element->tag_as_array("can_do", true);
    }
  });
  writer_can_do_[swid] = types;
  return true;
}

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
