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

#ifndef __SWITCHER_VRPN_ANALOG_SINK_DEVICE_H__
#define __SWITCHER_VRPN_ANALOG_SINK_DEVICE_H__

#include "./sink-device.hpp"
#include "vrpn_Analog.h"

namespace switcher {
namespace vrpn {

/**
 * Analog Sink Device
 */
class AnalogSinkDevice : public SinkDevice {
 public:
  AnalogSinkDevice(const std::string& name, int numChannels = 0);
  void start(VRPNConnection* connection);
  void stop();
  void loop();

  /**
   * Custom data save tree
   */
  InfoTree::ptr getTree() const;

  /**
   * Get the number of analog channels of the device
   * @return
   */
  int getNumChannels();

  /**
   * Set the number of analog channels of the device
   * @param numChannels
   */
  void setNumChannels(int numChannels);

  /**
   * Get the specified channel's value
   * @param index Channel index
   * @return Channel value
   */
  double getChannel(int index) const;

  /**
   * Set the specified channel's value
   * @param index Channel index
   * @param value Channel value
   */
  void setChannel(int index, double value);

 protected:
  /**
   * VRPN device server
   */
  std::unique_ptr<vrpn_Analog_Server> device_{};

  /**
   * Cached channelc ount
   */
  int numChannels_{};

  /**
   * Flag indicating if we have any changes pending that need to be sent
   */
  bool changed_{false};
};
}  // Namespace vrpn
}  // Namespace switcher

#endif