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

#ifndef __SWITCHER_VRPN_BUTTON_SINK_DEVICE_H__
#define __SWITCHER_VRPN_BUTTON_SINK_DEVICE_H__

#include "./button/vrpn-button-server.hpp"
#include "./sink-device.hpp"

namespace switcher {
namespace vrpn {

class ButtonSinkDevice : public SinkDevice {
 public:
  ButtonSinkDevice(const std::string& name, int numButtons = 0);
  void start(VRPNConnection* connection);
  void stop();
  void loop();

  /**
   * Custom data save tree
   */
  InfoTree::ptr getTree() const;

  /**
   * Get the number of buttons of the device
   * @return
   */
  int getNumButtons() const;

  /**
   * Set the number of buttons of the device
   * @param numButtons
   */
  void setNumButtons(int numButtons);

  /**
   * Get the specified button's state
   * @param index Button index
   * @return Button state
   */
  bool getButton(int index) const;

  /**
   * Set the specified button's state
   * @param index Button index
   * @param value Button value
   */
  void setButton(int index, bool value);

 protected:
  /**
   * VRPN device server
   */
  std::unique_ptr<VRPNButtonServer> device_{};

  /**
   * Cached button count
   */
  int numButtons_{};
};
}  // Namespace vrpn
}  // Namespace switcher

#endif