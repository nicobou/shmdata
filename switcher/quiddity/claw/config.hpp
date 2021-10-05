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

#ifndef __SWITCHER_QUIDDITY_CLAW_CONFIG_H__
#define __SWITCHER_QUIDDITY_CLAW_CONFIG_H__

#include "./types.hpp"

namespace switcher {
namespace quiddity {
namespace claw {

/**
 * claw::Config class is a minimal structure for argument passing
 * in Quiddity constructor, used eventually for the construction
 * of a Claw.
 *
 */
struct Config {
  /**
   * \brief JSON version of the specification to be parsed during Claw
   * construction.
   *
   */
  const std::string spec{};
  /**
   * \brief the callback to trigger when Claw is asked to connect
   * to a shmdata writer.
   *
   */
  const OnConnect_t on_connect_cb{};
  /**
   * \brief the callback to trigger when Claw is asked to disconnect
   * from a shmdata writer.
   *
   */
  const OnDisconnect_t on_disconnect_cb{};
};

}  // namespace claw
}  // namespace quiddity
}  // namespace switcher
#endif
