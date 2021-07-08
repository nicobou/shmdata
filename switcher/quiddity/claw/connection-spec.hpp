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

#ifndef __SWITCHER_QUIDDITY_CLAW_CONNECTION_SPEC_H__
#define __SWITCHER_QUIDDITY_CLAW_CONNECTION_SPEC_H__

#include <map>
#include <string>

#include "../../infotree/information-tree.hpp"
#include "../../utils/bool-log.hpp"
#include "./types.hpp"

namespace switcher {
namespace quiddity {
namespace claw {

/**
 * ConnectionSpec class maintains connection specification.
 * It uses an InfoTree and guaranties the well formed structure
 * of the specifications.
 *
 */
class ConnectionSpec : public BoolLog {
 public:
  /**
   * ConnectionSpec constructor. Start with an
   * empty specification.
   *
   */
  ConnectionSpec();

  /**
   * ConnectionSpec constructor. Parse the spec and
   * construct the Infotree accordingly.
   *
   * \param spec the specification in JSON format.
   *
   */
  ConnectionSpec(const std::string& spec);

  /**
   * Get Shmdata writer label from its identifier.
   *
   * \param swid the identifier
   *
   * \return the label
   */
  std::string get_writer_label(swid_t swid) const;

  /**
   * Get Shmdata follower label from its identifier.
   *
   * \param swid the identifier
   *
   * \return the label
   */
  std::string get_follower_label(sfid_t sfid) const;

  /**
   * Get Shmdata follower identifier from its label.
   *
   * \param label the label
   *
   * \return the identifier
   */
  sfid_t get_sfid(const std::string& label) const;

  /**
   * Get Shmdata writer identifier from its label.
   *
   * \param label the label
   *
   * \return the identifier
   */
  swid_t get_swid(const std::string& label) const;

  /**
   * Get Shmdata writer shmdata path from its identifier.
   *
   * \param id the identifier
   *
   * \return the Shmdata path
   */
  std::string get_writer_shmpath(swid_t id) const;

  /**
   * List labels of the Shmdata writers.
   *
   * \return the labels in a vector
   */
  std::vector<std::string> get_writer_labels() const;

  /**
   * List labels of the Shmdata followers.
   *
   * \return the labels in a vector
   */
  std::vector<std::string> get_follower_labels() const;

  /**
   * Test if an identifier is allocated.
   *
   * \param sfid the identifer
   *
   * \return true is allocated, false otherwise
   */
  bool is_allocated_follower(sfid_t sfid) const;

  /**
   * Test if the follower is a meta follower.
   *
   * \param sfid the follower identifier
   *
   * \return true if meta, false otherwise
   */
  bool is_meta_follower(sfid_t sfid) const;

  /**
   * Allocate a new identifier from a meta follower.
   *
   * \param sfid the meta follower identifier
   *
   * \return the newly allocated identifier, or Id::kInvalid
   */
  sfid_t allocate_sfid_from_meta(sfid_t sfid) const;

  /**
   * Deallocate an identifier allocated from a meta follower.
   * Also removes the follower description from the three.
   *
   * \param sfid the identifier to deallocate
   *
   * \return true if sfid has been deallocated, false if
   * sfid is not allocated, or not generated from a meta follower
   */
  bool deallocate_sfid_from_meta(sfid_t sfid) const;

  /**
   * Test if a Shmdata writer is allocated
   *
   * \param swid the Shmdata writer identifier
   *
   * \return true is allocated, false otherwise
   */
  bool is_allocated_writer(swid_t swid) const;

  /**
   * Test if a writer is meta.
   *
   * \param swid the writer identifier
   *
   * \return true if the write is meta, false otherwise
   */
  bool is_meta_writer(swid_t swid) const;

  /**
   * Allocate a writer from a meta writer.
   *
   * \param swid the identifier of the meta writer
   * \param spec the writer specification
   *
   * \return the identifier of newly allocated writer, of Id::kInvalid
   */
  swid_t allocate_swid_from_meta(swid_t swid, const shm_spec_t& spec);

  /**
   * Deallocate a writer allocated from a meta writer.
   * Also removes the writer from the connection specification.
   *
   * \param swid the identifier to deallocate
   *
   * return true if deallocated, false otherwise
   */
  bool deallocate_swid_from_meta(swid_t swid);

  /**
   * Get a reference to the internal InfomationTree.
   *
   * \return a ref-counted reference to the tree
   */
  InfoTree::ptr get_tree();

 private:
  mutable Ids follower_id_generator_{};
  mutable std::map<sfid_t, std::string> follower_ids_{};
  Ids writer_id_generator_{};
  std::map<swid_t, std::string> writer_ids_{};

  /**
   * Tree with structured specifications.
   */
  InfoTree::ptr connection_spec_;

  /**
   * Validate the shmdata specification.
   *
   * \param Tree containing the shmdata specification.
   *
   * \return Success with errors in the BoolLog message if any.
   */
  BoolLog check_shmdata_spec(const InfoTree* tree);
};

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
#endif
