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

#include "../../utils/safe-bool-idiom.hpp"

namespace switcher {
namespace quiddity {
namespace claw {

/**
 * ConnectionSpec class maintains connection specification.
 * It uses an InfoTree and guaranties the well formed structure
 * of the specifications.
 *
 */
class ConnectionSpec : public SafeBoolIdiom {
 public:
  /**
   * \brief ConnectionSpec constructor. Start with an
   * empty specification.
   *
   */
  ConnectionSpec();

  /**
   * \brief ConnectionSpec constructor. Parse the spec and
   * construct the Infotree accordingly.
   *
   * \param spec the specification in JSON format.
   *
   */
  ConnectionSpec(const std::string& spec);

 private:
  bool is_valid_{true};
  /**
   * \brief Implementation of the safe bool idiom. The ConnectionSpec
   * is valid if the constructor successfully parsed the initial spec.
   *
   */
  bool safe_bool_idiom() const;
};

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
#endif
