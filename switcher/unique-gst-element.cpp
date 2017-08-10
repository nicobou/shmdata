/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include "./unique-gst-element.hpp"

namespace switcher {
UGstElem::gst_element_handle::gst_element_handle(const std::string& class_name)
    : element(gst_element_factory_make(class_name.c_str(), nullptr)) {
  if (element) gst_object_ref(element);
}
UGstElem::gst_element_handle::~gst_element_handle() {
  if (element) {
    gst_object_unref(element);
    GstUtils::gst_element_deleter(element);
  }
};

BoolLog UGstElem::renew(UGstElem& element, const std::vector<std::string>& props) {
  // Unregister all property callbacks
  std::map<std::string, prop_notification_cb_t> registered_props;
  for (auto& notif : element.property_notifications_) {
    element.unregister_notify_on_property_change(notif.first);
    registered_props[notif.first] = notif.second.notification_cb;
  }

  std::unique_ptr<gst_element_handle> tmp =
      std::make_unique<gst_element_handle>(element.class_name_);
  if (!tmp) return BoolLog(false, "issue making tmp element");
  for (auto& it : props)
    GstUtils::apply_property_value(
        G_OBJECT(element.element_->get()), G_OBJECT(tmp->get()), it.c_str());
  std::swap(tmp, element.element_);

  // Re-register all property callbacks on the new GstElement.
  for (auto& prop : registered_props) {
    element.register_notify_on_property_change(prop.first, prop.second);
  }

  return BoolLog(true);
}

UGstElem::UGstElem(const gchar* class_name)
    : class_name_(class_name), element_(std::make_unique<gst_element_handle>(class_name)) {
}

bool UGstElem::safe_bool_idiom() const { return element_ && element_->get(); }

void UGstElem::g_invoke(std::function<void(gpointer)> command) {
  command(G_OBJECT(element_->get()));
  return;
}

void UGstElem::invoke(std::function<void(GstElement*)> command) {
  command(element_->get());
  return;
}

GstElement* UGstElem::get_raw() { return element_->get(); }

void UGstElem::mute(const gchar* class_name) { class_name_ = std::string(class_name); }

void UGstElem::property_notify_cb(GObject* /*gobject*/, GParamSpec* /*pspec*/, gpointer user_data) {
  auto content = static_cast<prop_notification_user_data_t*>(user_data);
  auto prop_notification = content->elem->property_notifications_.find(content->gprop_name);
  if (content->elem->property_notifications_.cend() != prop_notification)
    prop_notification->second.notification_cb();
}

BoolLog UGstElem::register_notify_on_property_change(const std::string& gprop_name,
                                                     prop_notification_cb_t callback) {
  auto object = element_->get();

  if (property_notifications_.cend() != property_notifications_.find(gprop_name)) {
    return BoolLog(
        false,
        std::string("Trying to register a notification for property ") + gprop_name +
            " while there is already one. Call unregister_notify_on_property_change() first.");
  }

  // Doing this to keep a valid reference to the object because this is going to the C realm.
  auto& prop_notification = property_notifications_[gprop_name];
  prop_notification.cb_user_data.elem = this;
  prop_notification.cb_user_data.gprop_name = gprop_name;

  unsigned long handler_id = g_signal_connect(G_OBJECT(object),
                                              std::string("notify::" + gprop_name).c_str(),
                                              G_CALLBACK(property_notify_cb),
                                              &prop_notification.cb_user_data);
  if (!handler_id) {
    property_notifications_.erase(gprop_name);
    return BoolLog(false, std::string("Could not register notification to property ") + gprop_name);
  }

  prop_notification.handler_id = handler_id;
  prop_notification.notification_cb = callback;

  return BoolLog(true);
}

BoolLog UGstElem::unregister_notify_on_property_change(const std::string& gprop_name) {
  auto notif = property_notifications_.find(gprop_name);
  if (property_notifications_.cend() == notif)
    return BoolLog(false,
                   std::string("Cannot unregister notification for property ") + gprop_name +
                       ", no notification was registered for it.");

  g_signal_handler_disconnect(G_OBJECT(element_->get()), notif->second.handler_id);
  property_notifications_.erase(notif);

  return BoolLog(true);
}
}  // namespace switcher
