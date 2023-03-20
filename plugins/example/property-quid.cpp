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

#include "property-quid.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PropertyQuid,
                                     "property-quid",
                                     "Example Quiddity with many property types",
                                     "Dummy plugin for testing/example purpose",
                                     "LGPL",
                                     "Nicolas Bouillot");

PropertyQuid::PropertyQuid(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      bool_id_(
          pmanage<MPtr(&property::PBag::make_bool)>("bool_",
                                                    [this](bool val) {
                                                      bool_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return bool_; },
                                                    "Bool Example",
                                                    "This property is an example for type bool",
                                                    bool_)),
      string_id_(
          pmanage<MPtr(&property::PBag::make_string)>("string_",
                                                      [this](const std::string& val) {
                                                        string_ = val;
                                                        return true;
                                                      },
                                                      [this]() { return string_; },
                                                      "String Example",
                                                      "This property is an example for type string",
                                                      string_)),
      char_id_(
          pmanage<MPtr(&property::PBag::make_char)>("char_",
                                                    [this](const char& val) {
                                                      char_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return char_; },
                                                    "Char Example",
                                                    "This property is an example for type char",
                                                    char_)),
      color_id_(
          pmanage<MPtr(&property::PBag::make_color)>("color_",
                                                     [this](const property::Color& val) {
                                                       color_ = val;
                                                       return true;
                                                     },
                                                     [this]() { return color_; },
                                                     "property::Color Example",
                                                     "This property is an example for type color",
                                                     color_)),
      integral_group_id_(pmanage<MPtr(&property::PBag::make_group)>(
          "integrals",
          "Integral Group Example",
          "This property is an example for grouping integral types")),
      int_id_(pmanage<MPtr(&property::PBag::make_parented_int)>(  // PBag factory
          "int_",                                                 // string id
          "integrals",                                            // parent
          [this](int val) {
            int_ = val;
            return true;
          },                                           // setter
          [this]() { return int_; },                   // getter
          "Int Example",                               // name
          "This property is an example for type int",  // description
          int_,                                        // default value
          -10,                                         // min
          10)),                                        // max
      short_id_(pmanage<MPtr(&property::PBag::make_parented_short)>(
          "short_",
          "integrals",
          [this](short val) {
            short_ = val;
            return true;
          },
          [this]() { return short_; },
          "Short Example",
          "This property is an example for type short",
          short_,
          -11,
          11)),
      long_id_(pmanage<MPtr(&property::PBag::make_parented_long)>(
          "long_",
          "integrals",
          [this](long val) {
            long_ = val;
            return true;
          },
          [this]() { return long_; },
          "Long Example",
          "This property is an example for type long",
          long_,
          -20,
          20)),
      long_long_id_(pmanage<MPtr(&property::PBag::make_parented_long_long)>(
          "long_long_",
          "integrals",
          [this](long long val) {
            long_long_ = val;
            return true;
          },
          [this]() { return long_long_; },
          "Long Long Example",
          "This property is an example for type long long",
          long_long_,
          -21,
          21)),
      unsigned_int_id_(pmanage<MPtr(&property::PBag::make_parented_unsigned_int)>(
          "unsigned_int_",
          "integrals",
          [this](unsigned int val) {
            unsigned_int_ = val;
            return true;
          },
          [this]() { return unsigned_int_; },
          "Unsigned Int Example",
          "This property is an example for type unsigned int",
          unsigned_int_,
          0,
          10)),
      unsigned_short_id_(pmanage<MPtr(&property::PBag::make_parented_unsigned_short)>(
          "unsigned_short_",
          "integrals",
          [this](unsigned short val) {
            unsigned_short_ = val;
            return true;
          },
          [this]() { return unsigned_short_; },
          "Unsigned Short Example",
          "This property is an example for type unsigned short",
          unsigned_short_,
          1,
          11)),
      unsigned_long_id_(pmanage<MPtr(&property::PBag::make_parented_unsigned_long)>(
          "unsigned_long_",
          "integrals",
          [this](unsigned long val) {
            unsigned_long_ = val;
            return true;
          },
          [this]() { return unsigned_long_; },
          "Unsigned Long Example",
          "This property is an example for type unsigned long",
          unsigned_long_,
          4,
          200)),
      unsigned_long_long_id_(pmanage<MPtr(&property::PBag::make_parented_unsigned_long_long)>(
          "unsigned_long_long_",
          "integrals",
          [this](unsigned long long val) {
            unsigned_long_long_ = val;
            return true;
          },
          [this]() { return unsigned_long_long_; },
          "Unsigned Long Long Example",
          "This property is an example for type unsigned long long",
          unsigned_long_long_,
          2,
          210)),
      floating_point_group_id_(pmanage<MPtr(&property::PBag::make_group)>(
          "floats",
          "Floating Point Group Example",
          "This property is an example for grouping floating points")),
      float_id_(pmanage<MPtr(&property::PBag::make_parented_float)>(
          "float_",
          "floats",
          [this](float val) {
            float_ = val;
            return true;
          },
          [this]() { return float_; },
          "Float Example",
          "This property is an example for type float",
          float_,
          -1.f,
          1.f)),
      double_id_(pmanage<MPtr(&property::PBag::make_parented_double)>(
          "double_",
          "floats",
          [this](double val) {
            double_ = val;
            return true;
          },
          [this]() { return double_; },
          "Double Example",
          "This property is an example for type double",
          double_,
          -1.f,
          10.f)),
      long_double_id_(pmanage<MPtr(&property::PBag::make_parented_long_double)>(
          "long_double_",
          "floats",
          [this](long double val) {
            long_double_ = val;
            return true;
          },
          [this]() { return long_double_; },
          "Long Double Example",
          "This property is an example for type long double",
          long_double_,
          -1.f,
          10.f)),
      selection_id_(pmanage<MPtr(&property::PBag::make_selection<>)>(
          "enum_",
          [this](const quiddity::property::IndexOrName& val) {
            selection_.select(val);
            return true;
          },
          [this]() { return selection_.get(); },
          "Selection Example",
          "This property is an example for type enum",
          selection_)),
      tuple_id_(pmanage<MPtr(&property::PBag::make_tuple<MyTuple>)>(
          "tuple_",
          [this](const MyTuple& val) {
            tuple_ = val;
            return true;
          },
          [this]() { return tuple_; },
          "Tuple Example",
          "This property is an example for tuple",
          tuple_)),
      fraction_id_(
          pmanage<MPtr(&property::PBag::make_fraction)>("fraction_",
                                                        [this](const property::Fraction& val) {
                                                          fraction_ = val;
                                                          return true;
                                                        },
                                                        [this]() { return fraction_; },
                                                        "property::Fraction Example",
                                                        "This property is an example for fraction",
                                                        fraction_,
                                                        -10,
                                                        1,  // min num/denom
                                                        10,
                                                        10)  // max num/denom
      ) {
  sw_debug("int_id_ is {}, unsigned_int_id_ is {}",
           std::to_string(pmanage<MPtr(&property::PBag::get<int>)>(int_id_)),
           std::to_string(pmanage<MPtr(&property::PBag::get<unsigned int>)>(unsigned_int_id_)));

  pmanage<MPtr(&property::PBag::set<MyTuple>)>(
      tuple_id_, std::make_tuple<long long, float, std::string>(2, 2.2, "a22"));

  sw_debug("tuple_ is {} {} {}",
           std::to_string(std::get<0>(tuple_)) /*2*/,
           std::to_string(std::get<1>(tuple_)) /* 2.2 */,
           std::get<2>(tuple_) /* a22*/);

  // creating some custom infos
  InfoTree::ptr tree = InfoTree::make();
  tree->graft(".child1.child2", InfoTree::make("switch"));
  tree->graft(".child1.child3", InfoTree::make(1.2f));
  tree->graft(".child1.child2.bla1", InfoTree::make("wire"));
  tree->graft(".child1.child2.bla2", InfoTree::make("hub"));
  // attaching it to the quiddity (at the root)
  graft_tree(".custom.information.", tree);
  sw_debug("hello from plugin");
}

}  // namespace quiddities
}  // namespace switcher
