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

#ifndef __SWITCHER_VRPN_SOURCE_DEVICE_H__
#define __SWITCHER_VRPN_SOURCE_DEVICE_H__

#include <map>
#include <memory>
#include "./properties/property-base.hpp"
#include "./properties/property.hpp"
#include "shared/connection/vrpn-connection.hpp"
#include "shared/device.hpp"
#include "vrpn_BaseClass.h"

namespace switcher {
namespace vrpn {

class SourceDevice : public Device {
 public:
  /**
   * Helper for property notification callbacks
   */
  using NotifyPropertyCallback = std::function<void(PContainer::prop_id_t propId)>;

  /**
   * Device Constructor
   *
   * @param connection
   * @param name
   * @param uri
   * @param notifyProperty
   * @return
   */
  SourceDevice(const std::string& name,
               const std::string& uri,
               NotifyPropertyCallback notifyProperty);

  virtual ~SourceDevice() = default;

  void loop();

  /**
   * Custom data save tree
   */
  InfoTree::ptr getTree() const;

 protected:
  /**
   * Device URI
   */
  std::string uri_;

  /**
   * Property Notification Callback
   */
  NotifyPropertyCallback notifyProperty_;

  /**
   * Raw VRPN Device
   */
  std::unique_ptr<vrpn_BaseClass> device_{};

  /**
   * Map of the VRPN Properties
   */
  std::map<std::string, std::unique_ptr<PropertyBase>> properties_{};

  /**
   * Utility for retrieving a property of the right type or nullptr
   *
   * @param id Id of the property
   * @return
   */
  template <typename T>
  Property<T>* getProperty(const std::string& id) {
    auto search = properties_.find(id);
    if (search != properties_.end()) {
      return dynamic_cast<Property<T>*>(search->second.get());
    } else {
      return nullptr;
    }
  }
};
}  // Namespace vrpn
}  // Namespace switcher
#endif
