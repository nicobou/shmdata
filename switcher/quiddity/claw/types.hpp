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

#ifndef __SWITCHER_QUIDDITY_CLAW_TYPES_H__
#define __SWITCHER_QUIDDITY_CLAW_TYPES_H__

#include <string>

#include "../../utils/ids.hpp"

namespace switcher {
namespace quiddity {
namespace claw {

/**
 * sfid_t is the type for unique identifier for a follower shmdata of a Quiddity.
 * It is allocated and maintained by in the Claw class.
 */
using sfid_t = Ids::id_t;

/**
 * swid_t is the type for unique identifier for a writer shmdata of a Quiddity.
 * It is allocated and maintained by in the Claw class.
 */
using swid_t = Ids::id_t;

/**
 * OnConnect_t is the callback triggered by Claw when asked to connect to
 * a shmdata writer.
 *
 * \param shmpath Path of the shmdata to connect to
 *
 * \param sid Identificator determined by the Claw. It can be used in order to
 * determine the specific shmdata that has been requested to connect to.
 *
 * \return Connection success
 */
using OnConnect_t = std::function<bool(const std::string& /*shmpath*/, sfid_t /*sid*/)>;

/**
 * OnDisconnect_t is the callback triggered by Claw when asked to disconnect
 * from a shmdata writer.
 *
 * \param sid Identificator of the shmdata to disconnect from.
 *
 * \return Disconnection success
 */
using OnDisconnect_t = std::function<bool(sfid_t /*sid*/)>;

/**
 * shm_spec_t is a data structure used for instanciation of dynamic shmdata
 */
using shm_spec_t = struct shm_spec_struct_t {
  std::string label{};        //!< The label for the Shmdata
  std::string description{};  //!< The description for the Shmdata
  std::vector<std::string> can_dos{};  //!< The can do specific to the new writer
};

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
#endif
