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

#ifndef __SWITCHER_PROPERTY_H__
#define __SWITCHER_PROPERTY_H__

#include <gst/gst.h>
#include <memory>
#include <map>
#include <string>
#include "./json-builder.hpp"
#include "./categorizable.hpp"

namespace switcher {
class Property:public Categorizable {
 public:
  typedef std::shared_ptr<Property> ptr;
  typedef void (*Callback) (GObject *gobject,
                            GParamSpec *pspec,
                            gpointer user_data);
  Property();
  ~Property();
  Property(const Property &source) = delete;
  Property &operator=(const Property &source) = delete;
  // this is when using an existing property
  void set_gobject_pspec(GObject * object, GParamSpec *pspec);
  void set_long_name(std::string long_name);
  void set_name(std::string name);
  void set(const std::string &value);
  std::string get();
  bool subscribe(Callback cb, void *user_data);
  bool unsubscribe(Callback cb, void *user_data);
  static std::string parse_callback_args(GObject *gobject, GParamSpec *pspec);
  GObject *get_gobject();
  GParamSpec *get_paramspec();
  std::string get_description();
  JSONBuilder::Node get_json_root_node();
  // void print();  // from gst-inspect
  // getter
  std::string get_long_name() const;
  std::string get_name() const;

 private:
  void make_description();
  std::string long_name_{};
  std::string name_{};
  GParamSpec *property_{nullptr};
  GObject *object_{nullptr};
  JSONBuilder::ptr json_description_;
  std::map<std::pair<Callback, void *>, gulong> subscribed_handlers_{};
};

}  // namespace switcher
#endif
