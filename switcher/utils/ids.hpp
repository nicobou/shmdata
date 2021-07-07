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

#ifndef __SWITCHER_UTILS_IDS_H__
#define __SWITCHER_UTILS_IDS_H__

#include <vector>

namespace switcher {

/**
 * Ids class maintains a list of unique numeric identifiers.
 *
 **/
class Ids {
 public:
  /**
   * id_t is the type of identifiers provided.
   *
   */
  using id_t = size_t;
  /**
   * kInvalid is the invalid identifier. It can be tester again other
   * identifiers with the operator "==".
   *
   */
  static const id_t kInvalid;
  /**
   * kMaxNumOfIds provides the maximum number of identifiers an Ids
   * instance can manage.
   *
   */
  static const id_t kMaxNumOfIds;

  /**
   * \brief Allocate a new identifier.
   *
   * \return The new identifier.
   */
  id_t allocate_id();
  /**
   * \brief Release the last allocated id.
   *
   */
  void release_last_allocated_id();
  /**
   * \brief Release an already allocated identifier.
   *
   * \param id The identifier to release.
   *
   * \return Success.
   */
  bool release_id(id_t id);
  /**
   * \brief Provide all allocated ids.
   *
   * \return The values allocated.
   */
  std::vector<id_t> get_ids() const;
  /**
   * \brief Test if an identifier is allocated.
   *
   * \param id the identificator to test.
   *
   * \return the allocation state.
   */
  bool is_allocated(id_t id) const;

 private:
  std::vector<id_t> ids_{};
  size_t cur_id_{kInvalid};
};

}  // namespace switcher
#endif
