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

#include <string>

#include "../../infotree/information-tree.hpp"
#include "../../utils/bool-log.hpp"

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

 private:
  /**
   * Tree with structured specifications.
   *
   */
  InfoTree::ptr connection_spec_;

  /**
   * Validate the shmdata specification.
   *
   * \param Tree containing the shmdata specification.
   *
   * \return Success with errors in the BoolLog message if any.
   *
   */
  BoolLog check_shmdata_spec(const InfoTree* tree);
};

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
#endif
