/**
 * This file is part of switcher-vrpn.
 *
 * switcher-vrpn is free software; you can redistribute it and/or
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

#include "vrpn-button-server.hpp"

namespace switcher {
namespace quiddities {
namespace vrpn {

VRPNButtonServer::VRPNButtonServer(const char* name, vrpn_Connection* connection, int numButtons)
    : vrpn_Button_Server(name, connection, numButtons) {}

vrpn_int32 VRPNButtonServer::getNumButtons(void) const { return num_buttons; }
vrpn_int32 VRPNButtonServer::setNumButtons(vrpn_int32 sizeRequested) {
  // This implementation was adapted from vrpn_Analog's equivalent
  // I don't know why they aren't both implemented in the same way
  if (sizeRequested < 0) sizeRequested = 0;
  if (sizeRequested > vrpn_BUTTON_MAX_BUTTONS) sizeRequested = vrpn_BUTTON_MAX_BUTTONS;
  num_buttons = sizeRequested;
  return num_buttons;
}
}  // namespace vrpn
}  // namespace quiddities
}  // namespace switcher
