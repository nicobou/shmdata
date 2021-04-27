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

#ifndef __SWITCHER_QUIDDITY_CLAW_CLAW_H__
#define __SWITCHER_QUIDDITY_CLAW_CLAW_H__

#include <cstdint>  // uint32_t
#include <functional>

#include "../../utils/safe-bool-idiom.hpp"
#include "./connection-spec.hpp"
#include "./types.hpp"

namespace switcher {
namespace quiddity {
class Quiddity;
namespace claw {

/**
 * Claw class provides Quiddity the ability to specify available
 * shmdata, both as writer and as follower. It is designed in
 * order to provide an API to Quiddity user, but also an API
 * for Quiddity authors.
 *
 * Claw offers an API for Quiddity user for connection to a
 * shmdata writer. It maintains a set of claw::sid_t that
 * identifies each unique shmdata points for the Quiddity.
 *
 * Claw provides introspection mechanism that provides shmdata
 * names and types, along with methods allowing for evaluation
 * of shmdata connection compatibility.
 *
 * Quiddity author, in order to enable a shmdata into switcher,
 * must provide a ConnectionSpec. The accompanying optional OnConnect_t
 * and OnDisconnect_t callbacks must be implemented for a Quiddity
 * that expects to read from shmdata.
 *
 */
class Claw : public SafeBoolIdiom {
 public:

  /**
   * \brief Claw constructor. It must be constructed by
   * the Quiddity.
   *
   * \param quid    Pointer to the Quiddity that owns the
   * Claw.
   * \param spec    The initial ConnectionSpec for the Quiddity
   * \param on_connect_cb the function to call when a Quiddity
   * is asked to connect to a shmdata writer
   * \param on_disconnect_cb the function to call when a Quiddity
   * is asked to disconnect to a shmdata writer
   *
   */
  Claw(Quiddity* quid,
       ConnectionSpec spec,
       OnConnect_t on_connect_cb,
       OnDisconnect_t on_disconnect_cb)
      : quid_(quid),
        connection_spec_(spec),
        on_connect_cb_(on_connect_cb),
        on_disconnect_cb_(on_disconnect_cb) {}

  // the following for a next MR
  // // reader quiddity
  // conid_t connect(sid_t local_sid, quiddity::qid_t writer_quid, sid_t writer_sid);
  // conid_t connect(sid_t local_sid, const std::string& shmpath);
  // bool disconnect(conid_t cid);

  // // name & sid converion connect using shmdata names
  // std::string get_name(sid_t writer_sid);
  // std::string get_shmpath(ShmdataId id);
  // sid_t get_sid(const std::string& name);

  // // dynamic specs
  // bool add_follower_spec(InfoTree::ptr spec);
  // bool add_writer_spec(InfoTree::ptr spec);
  // bool remove_follower_spec(const std::string& name);
  // bool remove_writer_spec(const std::string& name);

  // // test if connection is possible
  // sid_t get_first_compatible_sid(shmdat::Type caps);
  // std::vector<sid_t> get_compatible_sids(shmdat::Type caps);
  // bool can_sink_caps(shmdat::Type caps, sid_t local_sid);

 private:
  Quiddity* quid_;
  ConnectionSpec connection_spec_{};
  OnConnect_t on_connect_cb_{};
  OnDisconnect_t on_disconnect_cb_{};

  /**
   * \brief Implementation of the safe bool idioms. The Claw
   * is considered as valid if the Connection Spec has been
   * parsed with success
   *
   */
  bool safe_bool_idiom() const;
};

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
#endif
