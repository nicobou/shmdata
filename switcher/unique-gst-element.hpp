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

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "./bool-log.hpp"
#include "./gst-utils.hpp"
#include "./safe-bool-idiom.hpp"

namespace switcher {
class UGstElem : public SafeBoolIdiom {
 public:
  explicit UGstElem(const gchar* class_name);
  using prop_notification_cb_t = std::function<void()>;

  // invoke as g_object
  template <typename Return_type>
  Return_type g_invoke_with_return(std::function<Return_type(gpointer)> command) {
    return command(G_OBJECT(element_->get()));
  }
  void g_invoke(std::function<void(gpointer)> command);
  // invoke as GstElement
  template <typename Return_type>
  Return_type invoke_with_return(std::function<Return_type(GstElement*)> command) {
    return command(element_->get());
  }
  void invoke(std::function<void(GstElement*)> command);
  // get raw without taking ownership (do not unref)
  GstElement* get_raw();
  // renew
  static BoolLog renew(UGstElem& element, const std::vector<std::string>& props_to_apply = {});
  // mute (will be instanciated with a new class at renew)
  void mute(const gchar* class_name);

  static void property_notify_cb(GObject* /*gobject*/, GParamSpec* /*pspec*/, gpointer user_data);
  BoolLog register_notify_on_property_change(const std::string& gprop_name,
                                             prop_notification_cb_t callback);
  BoolLog unregister_notify_on_property_change(const std::string& gprop_name);

 private:
  struct gst_element_handle {
    gst_element_handle() = delete;
    gst_element_handle(const std::string& class_name);
    ~gst_element_handle();
    GstElement* get() const { return element; }
    GstElement* element{nullptr};
  };

  struct prop_notification_user_data_t {
    UGstElem* elem;
    std::string gprop_name;
  };

  struct prop_notification_t {
    unsigned long handler_id;
    prop_notification_cb_t notification_cb;
    prop_notification_user_data_t cb_user_data;
  };

  std::string class_name_;
  std::unique_ptr<gst_element_handle> element_;
  std::map<std::string, prop_notification_t> property_notifications_;
  // safe bool idiom implementation
  bool safe_bool_idiom() const final;
};

}  // namespace switcher
#endif
