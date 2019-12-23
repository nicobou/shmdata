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

#ifndef __SWITCHER_VRPN_DEVICE_H__
#define __SWITCHER_VRPN_DEVICE_H__

#include <string>
#include "./connection/vrpn-connection.hpp"
#include "switcher/infotree/information-tree.hpp"

namespace switcher {
namespace vrpn {

/**
 * Wrapper/Helper to be able to instanciate a server vrpn connection normally.
 */
class Device {
 public:
  Device() = delete;
  Device(const std::string& name);

  /**
   * Device name
   */
  std::string getName() const;

  /**
   * Custom data save tree
   */
  virtual InfoTree::ptr getTree() const;

  /**
   * Start the device
   * We pass the connection to use here since we need it to create the VRPN client/server for the
   * device.
   *
   * @param connection VRPNConnection used for the device client/server
   */
  virtual void start(VRPNConnection* connection) = 0;

  /**
   * Stop the device
   * The device's VRPN client/server needs to be reset in order to free the connection used by it.
   */
  virtual void stop() = 0;

  /**
   * Device's loop
   * Used to call whatever is necessary on the device's VRPN client/server
   */
  virtual void loop() = 0;

 protected:
  /**
   * Device name
   */
  std::string name_{};
};
}
}
#endif
