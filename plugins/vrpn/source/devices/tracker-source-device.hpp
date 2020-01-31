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

#ifndef __SWITCHER_VRPN_SOURCE_TRACKER_DEVICE_H__
#define __SWITCHER_VRPN_SOURCE_TRACKER_DEVICE_H__

#include "source-device.hpp"
#include "vrpn_Tracker.h"

namespace switcher {
namespace quiddities {
namespace vrpn {

struct TrackerPosision {
  double position[3];
  double quaternion[4];
};

class TrackerSourceDevice : public SourceDevice {
 public:
  TrackerSourceDevice(const std::string& name,
                      const std::string& uri,
                      SourceDevice::NotifyPropertyCallback notifyProperty);

  void start(VRPNConnection* connection);
  void stop();

  /**
   * Custom data save tree
   */
  InfoTree::ptr getTree() const;

 private:
  static void handleTrackerCallback(void* userData, const vrpn_TRACKERCB info);
};
}  // namespace vrpn
}  // namespace quiddities
}  // namespace switcher
#endif
