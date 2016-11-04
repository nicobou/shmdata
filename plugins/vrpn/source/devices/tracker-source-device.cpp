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

#include "tracker-source-device.hpp"

namespace switcher {
namespace vrpn {
TrackerSourceDevice::TrackerSourceDevice(const std::string& name,
                                         const std::string& uri,
                                         SourceDevice::NotifyPropertyCallback notifyProperty)
    : SourceDevice(name, uri, notifyProperty) {}

void TrackerSourceDevice::start(VRPNConnection* connection) {
  std::unique_ptr<vrpn_Tracker_Remote> tracker =
      std::make_unique<vrpn_Tracker_Remote>(uri_.c_str(), connection->raw());
  tracker->register_change_handler(this, handleTrackerCallback);
  device_ = std::move(tracker);
}

void TrackerSourceDevice::stop() { device_.reset(nullptr); }

InfoTree::ptr TrackerSourceDevice::getTree() const {
  InfoTree::ptr tree = SourceDevice::getTree();
  tree->graft("type", InfoTree::make("vrpn_Tracker"));
  return tree;
}

void TrackerSourceDevice::handleTrackerCallback(void* /*userData*/, const vrpn_TRACKERCB /*info*/) {
  // auto* context = static_cast<TrackerSourceDevice*>(userData);
}
}  // Namespace vrpn
}  // Namespace switcher
