/*
 * This file is part of switcher-myplugin.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_DUMMY_PLUGIN_H__
#define __SWITCHER_DUMMY_PLUGIN_H__

#include <memory>
#include <string>
// #include "switcher/startable-quiddity.hpp" FIXME re enable this
#include "switcher/quiddity.hpp"
#include "switcher/property2.hpp"
#include "switcher/selection.hpp"
#include "switcher/label.hpp"

namespace switcher {
class DummyPlugin: public Quiddity/*FIXME re enable this, public StartableQuiddity */ {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(DummyPlugin);
  DummyPlugin(const std::string &);
  ~DummyPlugin() = default;
  DummyPlugin(const DummyPlugin &) = delete;
  DummyPlugin &operator=(const DummyPlugin &) = delete;

 private:
  // --- Properties
  // the Property<T> object is providing access to the int_ member, accordingly
  // declare and initialise the member (e.g. int_) before the property (e.g. int_prop_) 
  // note: see switcher/type-name-registry.hpp for supported property types 
  int int_{3};
  PContainer::prop_id_t int_id_;
  // more examples
  unsigned int uint_{4};
  PContainer::prop_id_t uint_id_;
  bool bool_{true};
  PContainer::prop_id_t bool_id_;
  float float_{0.1234};
  PContainer::prop_id_t float_id_;
  double double_{4.321};
  PContainer::prop_id_t double_id_;
  std::string string_{"hello"};
  PContainer::prop_id_t string_id_;

  // selection
  Selection selection_{{"emacs", "vim", "notepad", "gedit"}, 0};
  PContainer::prop_id_t selection_id_;

  // Property grouping  // label can be added as parent
  PContainer::prop_id_t label_id_;
  
  // --- Methods
  std::string hello_{};
  static gchar *my_hello_world_method(gchar *first_arg, void *user_data);

  bool init() final;
  // bool start() final;
  // bool stop() final;
};

SWITCHER_DECLARE_PLUGIN(DummyPlugin);

}  // namespace switcher
#endif
