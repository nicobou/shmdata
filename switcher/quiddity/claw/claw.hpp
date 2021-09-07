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
#include <shmdata/type.hpp>
#include <vector>

#include "../../utils/safe-bool-idiom.hpp"
#include "../quid-id-t.hpp"
#include "./connection-spec.hpp"
#include "./types.hpp"

namespace switcher {
namespace quiddity {
class Quiddity;
namespace bundle {
class Bundle;
}  // namespace bundle

namespace claw {
/**
 * Claw class provides Quiddity the ability to specify available
 * shmdata, both as writer and as follower. It is designed in
 * order to provide an API to Quiddity user, but also an API
 * for Quiddity authors. It enables connection of Shmdata among
 * Quiddities, but also with raw Shmdata.
 *
 * Claw offers an API for Quiddity user for connection to a
 * shmdata writer. It maintains two sets of identifiers (writer and follower)
 * for each unique shmdata points for the Quiddity.
 *
 * Claw provides introspection mechanism that provides shmdata
 * labels and types, along with methods allowing for evaluation
 * of shmdata connection compatibility.
 *
 * Quiddity author, in order to enable a shmdata into switcher,
 * must provide a ConnectionSpec. The accompanying optional OnConnect_t
 * and OnDisconnect_t callbacks must be implemented for a Quiddity
 * that expects to read from shmdata.
 *
 * Both follower and writer can be "meta". Meta Shmdata have label containing a '%' character.
 * When meta, a Shmdata follower
 * or writer is not an actual Shmdata, but rather the description of a Shmdata generator.
 * For instance a meta writer "video%" may instantiate several video Shmdata, such as
 * "videoLeft", "videoRight", among others. While the number of generated Shmdata from
 * this meta writer is not know in advance, it is expected the actual data streams of generated
 * Shmdata comply with meta Shmdata "can_do" description.
 *
 * While a meta writer initiates new writers from inside the Quiddity implementation,
 * meta follower initiates new Shmdata follower when connect is invoked on the meta follower.
 * For instance, a recorder Quiddity could connect to an undefined number of audio Shmdata, and
 * then perform an aligned recording of these streams.
 */
class Claw : public SafeBoolIdiom {
  friend class bundle::Bundle;  // custom set of connection spec and callbacks
 public:
  /**
   * Claw constructor. A Claw object is designed to be owned and constructed by
   * the Quiddity. Its public const methods are exposed to the Quiddity user, through
   * Make_consultable. Other public methods are dedicated to Quiddity and other subclasses
   *
   * \param quid    Pointer to the Quiddity that owns the
   * Claw.
   * \param spec    The initial ConnectionSpec for the Quiddity
   * \param on_connect_cb the function to call when a Quiddity
   * is asked to connect to a shmdata writer
   * \param on_disconnect_cb the function to call when a Quiddity
   * is asked to disconnect to a shmdata writer
   */
  Claw(Quiddity* quid,
       ConnectionSpec spec,
       OnConnect_t on_connect_cb,
       OnDisconnect_t on_disconnect_cb);

  /**
   * Connect a local Shmdata follower with a Shmdata writer of an other Quiddity.
   *
   * \param local_sfid the Shmdata follower identifier to connect. If it is meta, a new follower
   * will be allocated from it. local_sfid must be allocated by the current Claw
   * \param writer_quid the identifier of the Quiddity hosting the Shmdata writer
   * \param writer_swid the identifier of the Shmdata writer, allocated by the Claw from the
   * Quiddity identified by writer_quid
   *
   * \return the identifier of the follower. A newly allocated identifier is generated if
   * local_sfid identifies a meta follower. If connect fails, Id::kInvalid will be returned.
   */
  sfid_t connect(sfid_t local_sfid, quiddity::qid_t writer_quid, swid_t writer_sid) const;

  /**
   * Connect a local Shmdata follower with an other Quiddity. The method is automatically finding
   * the first compatible shmdata writer available with the target Quiddity.
   *
   * \param local_sfid the Shmdata follower identifier to connect. If it is meta, a new follower
   * will be allocated from it. local_sfid must be allocated by the current Claw
   * \param writer_quid the identifier of the Quiddity hosting the Shmdata writer
   *
   * \return the identifier of the follower. A newly allocated identifier is generated if
   * local_sfid identifies a meta follower. If connect fails, Id::kInvalid will be returned.
   */
  sfid_t connect_quid(sfid_t local_sfid, quiddity::qid_t writer_quid) const;

  /**
   * Connect with an other Quiddity. The method is automatically finding
   * the first compatible shmdata writer available with the target Quiddity.
   *
   * \param writer_quid the identifier of the Quiddity hosting the Shmdata writer
   *
   * \return the identifier of the follower. A newly allocated identifier is generated if
   * local_sfid identifies a meta follower. If connect fails, Id::kInvalid will be returned.
   */
  sfid_t try_connect(quiddity::qid_t writer_quid) const;

  /**
   * Connect a local Shmdata follower with a Shmdata identified by its path.
   *
   * \param local_sfid the Shmdata follower identifier to connect. If it is meta, a new follower
   * will be allocated from it. local_sfid must be allocated by the current Claw
   * \param shmpath the Shmdata path to connect to
   *
   * \return the identifier of the follower. If connect fails, Id::kInvalid will be returned.
   */
  sfid_t connect_raw(sfid_t local_sfid, const std::string& shmpath) const;

  /**
   * Disconnect a Shmdata follower
   *
   * \param sfid the follower identifier
   *
   * \return true if success, false otherwise
   */
  bool disconnect(sfid_t sfid) const;

  /**
   * Get the label of a follower from its identifier
   *
   * \param sfid the follower identifier
   *
   * \return the label, or an empty string if sfid is not found
   */
  std::string get_follower_label(sfid_t sfid) const;

  /**
   * Get list of followers labels
   *
   * \return a vector with labels
   */
  std::vector<std::string> get_follower_labels() const;

  /**
   * Get identifier of a follower from its label
   *
   * \param label the follower label
   *
   * \return the identifier is found, Id::kInvalid otherwise
   */
  sfid_t get_sfid(const std::string& label) const;

  /**
   * Get list of follower identifiers
   *
   * \return the identifiers list
   */
  std::vector<sfid_t> get_sfids() const;

  /**
   * Get list of writers identifier
   *
   * \return the identifiers list
   */
  std::vector<swid_t> get_swids() const;

  /**
   * Get the label of a writer from its identifier
   *
   * \param swid the writer identifier
   *
   * \return the label, or an empty string if swid is not found
   */
  std::string get_writer_label(swid_t swid) const;

  /**
   * Get list of writers labels
   *
   * \return a vector with labels
   */
  std::vector<std::string> get_writer_labels() const;

  /**
   * Get identifier of a writer from its label
   *
   * \param label the writer label
   *
   * \return the identifier is found, Id::kInvalid otherwise
   */
  swid_t get_swid(const std::string& label) const;

  /**
   * Get the Shmdata path used by the writer
   *
   * \param swid the writer identifier
   *
   * \return the path, or empty string if swid is not found
   */
  std::string get_writer_shmpath(swid_t swid) const;

  /**
   * Get the Shmdata path from the writer label
   *
   * \param swid the writer identifier
   *
   * \return the path, or empty string if swid is not found
   */
  std::string get_shmpath_from_writer_label(const std::string& label) const;

  /**
   * Get shmdata follower ids compatible with a given shmdata type
   *
   * \param type the shmdata type
   *
   * \return the vector with all identifiers of compatible shmdata follower
   */
  std::vector<sfid_t> get_compatible_sfids(const ::shmdata::Type& type) const;

  /**
   * Get shmdata writer ids compatible with a given shmdata type
   *
   * \param type the shmdata type
   *
   * \return the vector with all identifiers of compatible shmdata writer
   */
  std::vector<swid_t> get_compatible_swids(const ::shmdata::Type& type) const;

  /**
   * Test if a local shmdata follower is compatible with the shmdata writer of a quiddity
   *
   * \param local_sfid the shmdata follower to test
   * \param writer_quid the writer Quiddity identifier
   * \param writer_swid the writer_quid's shmdata writer identifier
   *
   * \return true if compatible, false otherwise
   */
  bool can_do_swid(sfid_t local_sfid, quiddity::qid_t writer_quid, swid_t writer_swid) const;

  /**
   * Test if a local shmdata follower is compatible with a shmdata type
   *
   * \param local_sfid the shmdata follower to test
   * \param type the shmdata type
   *
   * \return true if compatible, false otherwise
   */
  bool sfid_can_do_shmtype(sfid_t local_sfid, const ::shmdata::Type& type) const;

  /**
   * Test if a local shmdata writer is compatible with a shmdata type
   *
   * \param local_swid the shmdata follower to test
   * \param type the shmdata type
   *
   * \return true if compatible, false otherwise
   */
  bool swid_can_do_shmtype(swid_t local_swid, const ::shmdata::Type& type) const;

  /**
   * Get the list of compatible types with a given shmdata follower
   *
   * \param sfid the shmdata follower identifier
   *
   * \return the list of compatible shmdata types
   */
  std::vector<::shmdata::Type> get_follower_can_do(sfid_t sfid) const;

  /**
   * Get the list of compatible types with a given shmdata writer
   *
   * \param sfid the shmdata writer identifier
   *
   * \return the list of compatible shmdata types
   */
  std::vector<::shmdata::Type> get_writer_can_do(swid_t swid) const;

  /**
   * Get the shmdata path a follower is connected to
   *
   * \param the shmdata follower identifier
   *
   * \return The shmdata path or empty string if not connected
   */
  std::string get_follower_shmpath(sfid_t sfid) const;

  /**
   * Add a writer corresponding to a meta writer specification.
   * It updates the connection specification.
   * This method is suposed to be used from Quiddity plugins.
   *
   * \param id the id of the meta Shmdata writer
   * \param spec the writer description (label + description).
   *        if the label field of the argument is is empty, then
   *        the claw will generate one
   *
   * \return the new Shmdata writer id
   */

  swid_t add_writer_to_meta(swid_t id, const shm_spec_t& spec);
  /*
   * Remove a writer from a meta writer specification.
   * It updates the connection specification.
   * This method is suposed to be used from Quiddity plugins.
   *
   * \param id the Shmdata writer to remove
   *
   * \return true is the writer has been removed, false otherwise.
   */
  bool remove_writer_from_meta(swid_t id);

  /**
   * Replace the follower can_do entry.
   *
   * \param sfid the follower identifier
   * \param types the shmdata types to apply
   *
   * \return applied or not
   */
  bool replace_follower_can_do(sfid_t sfid, const std::vector<::shmdata::Type>& types);

  /**
   * Replace the writer can_do entry.
   *
   * \param swid the writer identifier
   * \param types the shmdata types to apply
   *
   * \return applied or not
   */
  bool replace_writer_can_do(swid_t swid, const std::vector<::shmdata::Type>& types);

 private:
  Quiddity* quid_;
  ConnectionSpec connection_spec_{};
  OnConnect_t on_connect_cb_{};
  OnDisconnect_t on_disconnect_cb_{};
  mutable std::map<sfid_t, std::string> follower_shmpaths_{};
  std::map<swid_t, std::string> forced_writer_shmpaths_{};  //!< for use by bundle (friend class)
  /**
   * Implementation of the safe bool idioms. The Claw
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
