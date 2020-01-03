/*
 * This file is part of switcher-plugin-example.
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

#include "switcher/quiddity/quiddity.hpp"

// This quiddity implements an example of every possible property types for a quiddity

namespace switcher {
namespace quiddities {
using namespace quiddity;
class PropertyQuid : public Quiddity {
 public:
  PropertyQuid(quiddity::Config&&);
  ~PropertyQuid() = default;
  PropertyQuid(const PropertyQuid&) = delete;
  PropertyQuid& operator=(const PropertyQuid&) = delete;

 private:
  // the property::Property<T> object is providing access to the int_ member, accordingly
  // declare and initialise the member (e.g. bool_) before the property (e.g.
  // bool_id_)
  bool bool_{true};
  property::prop_id_t bool_id_;
  // an other property
  std::string string_{"hello"};
  property::prop_id_t string_id_;
  char char_{'@'};
  property::prop_id_t char_id_;
  property::Color color_{0, 255, 12, 123};
  property::prop_id_t color_id_;

  // grouping integral types
  property::prop_id_t integral_group_id_;
  int int_{3};
  property::prop_id_t int_id_;
  short short_{4};
  property::prop_id_t short_id_;
  long long_{5};
  property::prop_id_t long_id_;
  long long long_long_{6};
  property::prop_id_t long_long_id_;
  unsigned int unsigned_int_{3};
  property::prop_id_t unsigned_int_id_;
  unsigned short unsigned_short_{4};
  property::prop_id_t unsigned_short_id_;
  unsigned long unsigned_long_{5};
  property::prop_id_t unsigned_long_id_;
  unsigned long long unsigned_long_long_{6};
  property::prop_id_t unsigned_long_long_id_;

  // floating point types (also grouped)
  property::prop_id_t floating_point_group_id_;
  float float_{0.1234};
  property::prop_id_t float_id_;
  double double_{4.321};
  property::prop_id_t double_id_;
  long double long_double_{3.1};
  property::prop_id_t long_double_id_;

  // selection
  property::Selection<> selection_{{"emasc", "sublime", "IntelliJ IDEA", "vim", "notepad", "gedit"},
                                   0};
  property::prop_id_t selection_id_;

  // tuple
  using MyTuple = std::tuple<long long, float, std::string>;
  MyTuple tuple_{1, 3.14, "hello"};
  property::prop_id_t tuple_id_;

  // property::Fraction
  property::Fraction fraction_{1, 3};
  property::prop_id_t fraction_id_;
};

SWITCHER_DECLARE_PLUGIN(PropertyQuid);

}  // namespace quiddities
}  // namespace switcher
#endif
