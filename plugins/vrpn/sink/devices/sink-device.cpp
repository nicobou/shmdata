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

#include "./sink-device.hpp"

namespace switcher {
namespace vrpn {

SinkDevice::SinkDevice(const std::string& name) : Device(name) {}

InfoTree::ptr SinkDevice::getTree() const {
  // Only wrapped for now as we don't want concrete devices having to deal with
  // the fact that we implemented it or not.
  return Device::getTree();
}

}  // Namespace vrpn
}  // Namespace switcher
