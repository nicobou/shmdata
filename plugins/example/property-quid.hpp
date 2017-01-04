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

#include "switcher/quiddity.hpp"

namespace switcher {
class PropertyQuid : public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PropertyQuid);
  PropertyQuid(const std::string&);
  ~PropertyQuid() = default;
  PropertyQuid(const PropertyQuid&) = delete;
  PropertyQuid& operator=(const PropertyQuid&) = delete;

 private:
  // --- Properties
  // the Property<T> object is providing access to the int_ member, accordingly
  // declare and initialise the member (e.g. bool_) before the property (e.g.
  // bool_id_)
  bool bool_{true};
  PContainer::prop_id_t bool_id_;
  // an other property
  std::string string_{"hello"};
  PContainer::prop_id_t string_id_;
  char char_{'@'};
  PContainer::prop_id_t char_id_;
  Color color_{0, 255, 12, 123};
  PContainer::prop_id_t color_id_;

  // grouping integral types
  PContainer::prop_id_t integral_group_id_;
  int int_{3};
  PContainer::prop_id_t int_id_;
  short short_{4};
  PContainer::prop_id_t short_id_;
  long long_{5};
  PContainer::prop_id_t long_id_;
  long long long_long_{6};
  PContainer::prop_id_t long_long_id_;
  unsigned int unsigned_int_{3};
  PContainer::prop_id_t unsigned_int_id_;
  unsigned short unsigned_short_{4};
  PContainer::prop_id_t unsigned_short_id_;
  unsigned long unsigned_long_{5};
  PContainer::prop_id_t unsigned_long_id_;
  unsigned long long unsigned_long_long_{6};
  PContainer::prop_id_t unsigned_long_long_id_;

  // floating point types (also grouped)
  PContainer::prop_id_t floating_point_group_id_;
  float float_{0.1234};
  PContainer::prop_id_t float_id_;
  double double_{4.321};
  PContainer::prop_id_t double_id_;
  long double long_double_{3.1};
  PContainer::prop_id_t long_double_id_;

  // selection
  Selection<> selection_{{"emasc", "sublime", "IntelliJ IDEA", "vim", "notepad", "gedit"}, 0};
  PContainer::prop_id_t selection_id_;

  // tuple
  // using MyTuple = std::tuple<long long, float, std::string>;
  // MyTuple tuple_{1, 3.14, "hello"};
  // PContainer::prop_id_t tuple_id_;

  // Fraction
  Fraction fraction_{1, 3};
  PContainer::prop_id_t fraction_id_;

  // --- Methods
  std::string hello_{};
  static gchar* my_hello_world_method(gchar* first_arg, void* user_data);

  bool init() final;
};

SWITCHER_DECLARE_PLUGIN(PropertyQuid);

}  // namespace switcher
#endif
