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

#include "./claw.hpp"

#include "../../switcher.hpp"
#include "../../utils/scope-exit.hpp"
#include "../quiddity.hpp"

namespace switcher {
namespace quiddity {
namespace claw {

bool Claw::safe_bool_idiom() const { return static_cast<bool>(connection_spec_); }

Claw::Claw(Quiddity* quid,
           ConnectionSpec spec,
           OnConnect_t on_connect_cb,
           OnDisconnect_t on_disconnect_cb)
    : quid_(quid),
      connection_spec_(spec),
      on_connect_cb_(on_connect_cb),
      on_disconnect_cb_(on_disconnect_cb) {
  // check specification has been parsed
  if (!static_cast<bool>(connection_spec_)) {
    quid_->warning("Connection spec error: %", connection_spec_.msg());
    return;
  }
  // check callbacks for follower connection are installed properly
  if (!connection_spec_.get_follower_labels().empty()) {
    if (!on_connect_cb_) {
      quid_->warning(
          "a follower is specified in connection spec, but no connection callback is configured");
      return;
    }
    if (!on_disconnect_cb_) {
      quid_->warning(
          "a follower is specified in connection spec, but no disconnection callback is "
          "configured");
      return;
    }
  }
  quid_->connection_spec_tree_ = connection_spec_.get_tree();
}

std::string Claw::get_follower_label(sfid_t sfid) const {
  return connection_spec_.get_follower_label(sfid);
}

std::string Claw::get_writer_label(swid_t swid) const {
  return connection_spec_.get_writer_label(swid);
}

sfid_t Claw::get_sfid(const std::string& label) const { return connection_spec_.get_sfid(label); }

swid_t Claw::get_swid(const std::string& label) const { return connection_spec_.get_swid(label); }

sfid_t Claw::connect(sfid_t local_sid, quiddity::qid_t writer_quid, swid_t writer_sid) const {
  auto writer_qrox = quid_->qcontainer_->get_qrox(writer_quid);
  if (!writer_qrox) {
    quid_->warning("Quiddity % not found, cannot connect", std::to_string(writer_quid));
    return Ids::kInvalid;
  }
  auto shmpath = writer_qrox.get()->claw_.get_writer_shmpath(writer_sid);
  if (shmpath.empty()) {
    quid_->warning("Shmdata writer % not found, cannot connect", std::to_string(writer_sid));
    return Ids::kInvalid;
  }
  return connect_raw(local_sid, shmpath);
}

sfid_t Claw::connect_raw(sfid_t sfid, const std::string& shmpath) const {
  auto id = sfid;
  bool is_meta = connection_spec_.is_meta_follower(sfid);
  if (is_meta) {
    id = connection_spec_.allocate_sfid_from_meta(sfid);
  }
  if (!connection_spec_.is_allocated_follower(id)) {
    quid_->warning("cannot connect_raw, shmdata follower not found (id requested is %)",
                   std::to_string(sfid));
    return Ids::kInvalid;
  }
  // trigger quiddity callback
  if (!(on_connect_cb_(shmpath, id))) {
    if (is_meta) {
      connection_spec_.deallocate_sfid_from_meta(id);
    }
    return Ids::kInvalid;
  }
  // notify new follower
  if (is_meta) {
    quid_->smanage<MPtr(&signal::SBag::notify)>(quid_->on_connection_spec_grafted_id_,
                                                InfoTree::make("follower." + std::to_string(id)));
  }
  return id;
}

bool Claw::disconnect(sfid_t sfid) const {
  if (!connection_spec_.is_allocated_follower(sfid)) {
    quid_->warning("cannot disconnect (sfid % not allocated)", std::to_string(sfid));
    return false;
  }
  if (!(on_disconnect_cb_(sfid))) {
    return false;
  }
  // remove from connection spec if a follower is generated from a meta follower
  if (connection_spec_.deallocate_sfid_from_meta(sfid)) {
    quid_->smanage<MPtr(&signal::SBag::notify)>(quid_->on_connection_spec_pruned_id_,
                                                InfoTree::make("follower." + std::to_string(sfid)));
  }
  return true;
}

std::string Claw::get_writer_shmpath(swid_t id) const {
  if (!connection_spec_.is_allocated_writer(id)) {
    quid_->warning("cannot provide shmpath for id % (not allocated)", std::to_string(id));
    return "";
  }
  if (!connection_spec_.is_meta_writer(id)) {
    quid_->warning("cannot provide shmpath for id % (id refers to a meta Shmdata)",
                   std::to_string(id));
    return "";
  }
  return Switcher::get_shm_dir() + "/" + Switcher::get_shm_prefix() +
         quid_->qcontainer_->get_switcher()->get_name() + "_" + std::to_string(quid_->id_) + "_" +
         std::to_string(id);
}

swid_t Claw::add_writer_to_meta(swid_t swid, const shm_spec_t& spec) {
  auto id = connection_spec_.allocate_swid_from_meta(swid, spec);
  if (Ids::kInvalid == id) {
    return id;
  }
  quid_->smanage<MPtr(&signal::SBag::notify)>(quid_->on_connection_spec_grafted_id_,
                                              InfoTree::make("writer." + std::to_string(id)));
  return id;
}

bool Claw::remove_writer_from_meta(swid_t swid) {
  auto id = connection_spec_.deallocate_swid_from_meta(swid);
  if (Ids::kInvalid == id) {
    return id;
  }
  quid_->smanage<MPtr(&signal::SBag::notify)>(quid_->on_connection_spec_pruned_id_,
                                              InfoTree::make("writer." + std::to_string(id)));
  return id;
}

std::vector<std::string> Claw::get_follower_labels() const {
  return connection_spec_.get_follower_labels();
}

std::vector<std::string> Claw::get_writer_labels() const {
  return connection_spec_.get_writer_labels();
}

bool Claw::can_do_shmtype(sfid_t sfid, const ::shmdata::Type& shmtype) const {
  const auto& can_do = connection_spec_.get_follower_can_do(sfid);
  auto writer_caps = gst_caps_from_string(shmtype.str().c_str());
  On_scope_exit { gst_caps_unref(writer_caps); };
  for (const auto& it : can_do) {
    auto can_do_caps = gst_caps_from_string(it.str().c_str());
    On_scope_exit { gst_caps_unref(can_do_caps); };
    if (gst_caps_is_always_compatible(can_do_caps, writer_caps)) {
      return true;
    }
  }
  return false;
}

bool Claw::can_do_swid(sfid_t local_sfid, quiddity::qid_t writer_quid, swid_t writer_sid) const {
  auto writer_qrox = quid_->qcontainer_->get_qrox(writer_quid);
  if (!writer_qrox) {
    quid_->warning("Quiddity % not found", std::to_string(writer_quid));
    return false;
  }
  auto writer_can_do = writer_qrox.get()->claw_.get_writer_can_do(writer_sid);
  for (const auto& can_do : writer_can_do) {
    if (can_do_shmtype(local_sfid, can_do)) {
      return true;
    }
  }
  return false;
}

std::vector<sfid_t> Claw::get_compatible_sfids(const ::shmdata::Type& type) const {
  std::vector<sfid_t> res;
  for (const auto& it : connection_spec_.get_follower_sfids()) {
    if (can_do_shmtype(it, type)) {
      res.emplace_back(it);
    }
  }
  return res;
}

std::vector<swid_t> Claw::get_compatible_swids(const ::shmdata::Type& type) const {
  std::vector<swid_t> res;
  for (const auto& it : connection_spec_.get_writer_swids()) {
    if (can_do_shmtype(it, type)) {
      res.emplace_back(it);
    }
  }
  return res;
}

std::vector<::shmdata::Type> Claw::get_follower_can_do(sfid_t sfid) const {
  return connection_spec_.get_follower_can_do(sfid);
}

std::vector<::shmdata::Type> Claw::get_writer_can_do(swid_t swid) const {
  return connection_spec_.get_writer_can_do(swid);
}

sfid_t Claw::connect_quid(sfid_t local_sfid, quiddity::qid_t writer_quid) const {
  auto writer_qrox = quid_->qcontainer_->get_qrox(writer_quid);
  if (!writer_qrox) {
    quid_->warning("Quiddity % not found, connect connect", std::to_string(writer_quid));
    return Ids::kInvalid;
  }
  swid_t swid = Ids::kInvalid;
  for (const auto& it : get_follower_can_do(local_sfid)) {
    auto swids = writer_qrox.get()->claw_.get_compatible_swids(it);
    if (!swids.empty()) {
      swid = swids[0];
      break;
    }
  }
  if (Ids::kInvalid == swid) {
    quid_->warning("No compatible shmdata found, conneot connect with %",
                   std::to_string(writer_quid));
    return Ids::kInvalid;
  }
  return connect(local_sfid, writer_quid, swid);
}

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
