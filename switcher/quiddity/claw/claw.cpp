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
  if (!static_cast<bool>(connection_spec_)) {
    quid->warning("Connection spec error: %", connection_spec_.msg());
    return;
  }
  quid->connection_spec_tree_ = connection_spec_.get_tree();
}

std::string Claw::get_follower_name(sfid_t sfid) const {
  return connection_spec_.get_follower_name(sfid);
}

std::string Claw::get_writer_name(swid_t swid) const {
  return connection_spec_.get_writer_name(swid);
}

sfid_t Claw::get_sfid(const std::string& name) const { return connection_spec_.get_sfid(name); }

swid_t Claw::get_swid(const std::string& name) const { return connection_spec_.get_sfid(name); }

sfid_t Claw::connect(sfid_t local_sid, quiddity::qid_t writer_quid, swid_t writer_sid) {
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

sfid_t Claw::connect_raw(sfid_t sfid, const std::string& shmpath) {
  auto id = connection_spec_.get_actual_sfid(sfid);
  if (Ids::kInvalid == id) {
    quid_->warning("cannot connect, shmdata follower not found (id requested is %)",
                   std::to_string(sfid));
    return Ids::kInvalid;
  }
  // trigger quiddity callback
  if (!(on_connect_cb_(shmpath, id))) {
    connection_spec_.release(id);
    return Ids::kInvalid;
  }
  return id;
}

bool Claw::disconnect(sfid_t sfid) {
  if (!connection_spec_.is_allocated(sfid)) {
    quid_->warning("cannot disconnect (sfid % not allocated)", std::to_string(sfid));
    return false;
  }
  if (!(on_disconnect_cb_(sfid))) {
    return false;
  }
  return true;
}

std::string Claw::get_writer_shmpath(swid_t id) const {
  if (!connection_spec_.is_actual_writer(id)) {
    quid_->warning("cannot provide shmpath for id % (id not valid for writer)", std::to_string(id));
    return "";
  }
  return Switcher::get_shm_dir() + "/" + Switcher::get_shm_prefix() +
         quid_->qcontainer_->get_switcher()->get_name() + "_" + std::to_string(quid_->id_) + "_" +
         std::to_string(id);
}

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
