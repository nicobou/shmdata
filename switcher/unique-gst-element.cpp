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

bool UGstElem::renew(UGstElem& element, const std::vector<std::string>& props) {
  g_debug("renewing gst element of class %s", element.class_name_.c_str());
  std::unique_ptr<gst_element_handle> tmp =
      std::make_unique<gst_element_handle>(element.class_name_);
  if (!tmp) return false;
  for (auto& it : props)
    GstUtils::apply_property_value(
        G_OBJECT(element.element_->get()), G_OBJECT(tmp->get()), it.c_str());
  std::swap(tmp, element.element_);
  return true;
}

UGstElem::UGstElem(const gchar* class_name)
    : class_name_(class_name), element_(std::make_unique<gst_element_handle>(class_name)) {
  if (!element_) g_warning("GStreamer element cannot be created (type %s)", class_name);
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

}  // namespace switcher
