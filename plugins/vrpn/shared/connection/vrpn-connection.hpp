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

#ifndef __SWITCHER_VRPN_CONNECTION_H__
#define __SWITCHER_VRPN_CONNECTION_H__

#include <vrpn_Connection.h>

namespace switcher {
namespace quiddities {
namespace vrpn {

/**
 * Wrapper for the vrpn_Connection pointer
 */
class VRPNConnection {
 public:
  VRPNConnection() = delete;
  VRPNConnection(vrpn_Connection* connection) : connection_(connection){};
  ~VRPNConnection() {
    if (connection_) {
      connection_->removeReference();
    }
  }
  /**
   * Raw vrpn_Connection pointer
   * Use only when passing the connection to vrpn as a pointer is required.
   */
  vrpn_Connection* raw() { return connection_; }

 private:
  vrpn_Connection* connection_;
};
}  // namespace vrpn
}  // namespace quiddities
}  // namespace switcher

#endif
