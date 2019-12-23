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

#ifndef __SWITCHER_VRPN_SINK_DEVICE_H__
#define __SWITCHER_VRPN_SINK_DEVICE_H__

#include <memory>
#include <string>
#include <switcher/infotree/information-tree.hpp>
#include "shared/device.hpp"

namespace switcher {
namespace vrpn {

class SinkDevice : public Device {
 public:
  SinkDevice() = delete;
  SinkDevice(const std::string& name);

  virtual ~SinkDevice() = default;

  /**
   * Custom data save tree
   */
  InfoTree::ptr getTree() const;
};
}  // Namespace vrpn
}  // Namespace switcher

#endif
