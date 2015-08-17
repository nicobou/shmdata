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

#ifndef __SWITCHER_UNIQUE_GST_ELEMENT_H__
#define __SWITCHER_UNIQUE_GST_ELEMENT_H__

#include <memory>
#include <functional>
#include <string>
#include <vector>
#include "./safe-bool-idiom.hpp"
#include "./gst-utils.hpp"

namespace switcher {
class UGstElem : public SafeBoolIdiom {
 public:
  explicit UGstElem(const gchar *class_name);
  // invoke as g_object
  template<typename Return_type>
  Return_type
  g_invoke_with_return(std::function<Return_type(gpointer)> command) {
    return command(G_OBJECT(element_.get()));
  }
  void g_invoke(std::function<void(gpointer)> command);
  // invoke as GstElement
  template<typename Return_type>
  Return_type
  invoke_with_return(std::function<Return_type(GstElement *)> command) {
    return command(element_.get());
  }
  void invoke(std::function<void(GstElement *)> command);
  // get raw without taking ownership (do not unref)
  GstElement *get_raw();
  // renew
  static bool renew(UGstElem &element, const std::vector<std::string> &props_to_apply = {});
  // mute (will be instanciated with a new class at renew)
  void mute(const gchar *class_name);
 private:
  std::string class_name_{};
  using gst_element_handle =
      std::unique_ptr<GstElement, decltype(&GstUtils::gst_element_deleter)>;
  gst_element_handle element_;
  // safe bool idiom implementation
  bool safe_bool_idiom() const final;
};
}  // namespace switcher

#endif
