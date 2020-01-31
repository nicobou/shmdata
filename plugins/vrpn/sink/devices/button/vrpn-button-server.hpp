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

#ifndef __SWITCHER_VRPN_BUTTON_SERVER_H__
#define __SWITCHER_VRPN_BUTTON_SERVER_H__

#include "vrpn_Button.h"

namespace switcher {
namespace quiddities {
namespace vrpn {

/**
 * Overriden because the original doesn't let us set the number of buttons
 * nor get the values of the buttons
 */
class VRPNButtonServer : public vrpn_Button_Server {
 public:
  VRPNButtonServer(const char* name, vrpn_Connection* connection, int numButtons = 0);

  /**
   * Getter for the buttons, since it it not exposed by the parent class
   * @return
   */
  unsigned char* getButtons(void) { return buttons; }

  /**
   * Get the number of buttons
   * @return
   */
  vrpn_int32 getNumButtons(void) const;

  /**
   * Set the number of buttons
   * @param sizeRequested
   * @return
   */
  vrpn_int32 setNumButtons(vrpn_int32 sizeRequested);
};

}  // namespace vrpn
}  // namespace quiddities
}  // namespace switcher
#endif
