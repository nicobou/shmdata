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

#include "./device.hpp"

namespace switcher {
namespace vrpn {

Device::Device(const std::string& name) : name_(name) {}

std::string Device::getName() const { return name_; };

InfoTree::ptr Device::getTree() const {
  InfoTree::ptr tree = InfoTree::make();
  tree->graft("name", InfoTree::make(name_));
  return tree;
}

}  // Namespace vrpn
}  // Namespace switcher
