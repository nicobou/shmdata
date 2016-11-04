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

#ifndef __SWITCHER_VRPN_SOURCE_BUTTON_DEVICE_H__
#define __SWITCHER_VRPN_SOURCE_BUTTON_DEVICE_H__

#include "source-device.hpp"
#include "vrpn_Button.h"

namespace switcher {
namespace vrpn {

class ButtonSourceDevice : public SourceDevice {
 public:
  /**
   * Type for the bool property creation callback
   */
  using CreateBoolPropertyCallback =
      std::function<PContainer::prop_id_t(Property<bool>* property,
                                          const std::string& id,
                                          const std::string& name,
                                          const std::string& description,
                                          bool value)>;
  ButtonSourceDevice(const std::string& name,
                     const std::string& uri,
                     int numProperties,
                     CreateBoolPropertyCallback createBoolProperty,
                     SourceDevice::NotifyPropertyCallback notifyProperty);

  void start(VRPNConnection* connection);
  void stop();

  /**
   * Custom data save tree
   */
  InfoTree::ptr getTree() const;

 private:
  /**
   * Bool property creation callback
   */
  CreateBoolPropertyCallback createBoolProperty_;
  void handleButton(int index, bool value);
  static void handleButtonCallback(void* userData, const vrpn_BUTTONCB info);
};
}  // Namespace vrpn
}  // Namespace switcher
#endif
