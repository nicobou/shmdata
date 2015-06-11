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
#include "./std2.hpp"

namespace switcher {
bool UGstElem::renew(UGstElem &element, const std::vector<std::string> &props) {
  g_debug("renewing gst element of class %s", element.class_name_.c_str());
  gst_element_handle tmp(gst_element_factory_make(element.class_name_.c_str(),
                                                  nullptr),
                         &GstUtils::gst_element_deleter);
  for (auto &it: props)
    GstUtils::apply_property_value(G_OBJECT(element.element_.get()),
                                   G_OBJECT(tmp.get()),
                                   it.c_str());
  if (!tmp)
    return false;
  std::swap(tmp, element.element_);
  return true;
}

UGstElem::UGstElem(const gchar *class_name):
    class_name_(class_name),
    element_(gst_element_factory_make(class_name, nullptr),
             &GstUtils::gst_element_deleter) {
}

bool UGstElem::safe_bool_idiom() const {
  return static_cast<bool>(element_);
}

void UGstElem::g_invoke(std::function<void(gpointer)> command) {
  command(G_OBJECT(element_.get()));
  return;
}

void UGstElem::invoke(std::function<void(GstElement *)> command) {
  command(element_.get());
  return;
}

GstElement *UGstElem::get_raw() {
  return element_.get();
}

void UGstElem::mute(const gchar *class_name) {
  class_name_ = std::string(class_name);
}

}  // namespace switcher
