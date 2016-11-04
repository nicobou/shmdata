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

#include "./analog-sink-device.hpp"

namespace switcher {
namespace vrpn {

AnalogSinkDevice::AnalogSinkDevice(const std::string& name, int numChannels)
    : SinkDevice(name), numChannels_(numChannels) {}

void AnalogSinkDevice::start(VRPNConnection* connection) {
  device_ = std::make_unique<vrpn_Analog_Server>(name_.c_str(), connection->raw(), numChannels_);
}

void AnalogSinkDevice::stop() { device_.reset(nullptr); }

void AnalogSinkDevice::loop() {
  if (device_) {
    if (changed_) {
      device_->report_changes();
      changed_ = false;
    }
    device_->mainloop();
  }
}

InfoTree::ptr AnalogSinkDevice::getTree() const {
  InfoTree::ptr tree = SinkDevice::getTree();
  tree->graft("type", InfoTree::make("vrpn_Analog"));
  tree->graft("numChannels", InfoTree::make(numChannels_));
  return tree;
}

int AnalogSinkDevice::getNumChannels() { return numChannels_; }

void AnalogSinkDevice::setNumChannels(int numChannels) {
  numChannels_ = numChannels;
  if (device_) {
    device_->setNumChannels(numChannels_);
  }
}

double AnalogSinkDevice::getChannel(int index) const {
  return device_ ? device_->channels()[index] : 0;
}

void AnalogSinkDevice::setChannel(int index, double value) {
  if (device_) {
    device_->channels()[index] = value;
    changed_ = true;
  }
}

}  // Namespace vrpn
}  // Namespace switcher
