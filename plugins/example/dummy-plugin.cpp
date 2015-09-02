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

#include "./dummy-plugin.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    DummyPlugin,
    "dummy",
    "Dummy Plugin",
    "test",
    "",
    "Dummy plugin for testing/example purpose",
    "LGPL",
    "Nicolas Bouillot");

DummyPlugin::DummyPlugin(const std::string &) :
    // for arythmetic types, min and max specifications are optionnal,
    // if not specified, std::numeric_limits<T>::value is used
    int_id_(prop_do(&PContainer::make_int,                       // PContainer maker
                     "int_",                                      // string id
                     [this](int val){int_ = val; return true;},   // setter
                     [this](){return int_;},                      // getter
                     "Int Example",                               // name
                     "This property is an example for type int",  // description
                     int_,                                           // default value
                     -10,                                         // min
                     10)),                                         // max
  uint_id_(prop_do(&PContainer::make_unsigned_int,                       // PContainer maker
                    "uint_",                                              // string id
                    [this](unsigned int val){uint_ = val; return true;},  // setter
                    [this](){return uint_;},                              // getter
                    "Unsigned Int Example",                               // name
                    "This property is an example for type unsigned int",  // description
                    uint_,                                                // default value
                    1,                                                    // min 
                    4)),                                                  // max
    bool_id_(prop_do(&PContainer::make_bool,
                      "bool_",
                      [this](bool val){bool_ = val; return true;},
                      [this](){return bool_;},
                      "Bool Example",
                      "This property is an example for type bool",
                      bool_)),
  float_id_(prop_do(&PContainer::make_float,
                     "float_",
                     [this](float val){float_ = val; return true;},
                     [this](){return float_;},
                     "Float Example",
                     "This property is an example for type float",
                     float_,
                     -1.f,
                     1.f)),
  double_id_(prop_do(&PContainer::make_double,
                      "double_",
                      [this](double val){double_ = val; return true;},
                      [this](){return double_;},
                      "Double Example",
                      "This property is an example for type double",
                      double_,
                      -1.d,
                      10.d)),
  string_id_(prop_do(&PContainer::make_string,
                      "string_",
                      nullptr, //[this](const std::string &val){string_ = val; return true;},
                      [this](){return string_;},
                      "String Example",
                      "This property is an example for type string",
                      string_)),
  selection_id_(prop_do(&PContainer::make_selection,
                         "enum_",
                         [this](size_t val){selection_.select(val); return true;},
                         [this](){return selection_.get();},
                         "Selection Example",
                         "This property is an example for type enum",
                         selection_)),
  label_id_(prop_do(&PContainer::make_label,
                    "label_",
                    "Label Example",
                    "This property is an example for label")),
  tuple_id_(prop_do(&PContainer::make_tuple<long long, float, std::string>,
                    "tuple_",
                    [this](const std::tuple<long long, float, std::string> &val){
                      tuple_ = val; return true;
                    },
                    [this](){return tuple_;},
                    "Tuple Example",
                    "This property is an example for tuple",
                    tuple_)
            ){
  std::cout << prop_do(&PContainer::get<int>, int_id_) << std::endl;
  std::cout << prop_do(&PContainer::get<unsigned int>, uint_id_) << std::endl;

  prop_do(&PContainer::set<std::tuple<long long, float, std::string>>,
          tuple_id_,
          std::make_tuple<long long, float, std::string>(2,2.2,"a22"));

  std::cout << std::get<0>(tuple_) << " "      // 2
            << std::get<1>(tuple_) << " "      // 2.2
            << std::get<2>(tuple_) << "\n";    // a22
}

bool DummyPlugin::init() {
  
  // g_debug("uint property installation id is %lu", uint_id);
  // props_.install("int_", &int_prop_);  
  install_method("Hello World",  // long name
                 "hello-world",  // name
                 "say hello and repeat first argument",  // description
                 "the hello answer",  // return description
                 Method::make_arg_description("Text To Repeat",       // first arg long name
                                              "text",  // fisrt arg name
                                              "string",       // first arg description
                                              nullptr),
                 (Method::method_ptr) &my_hello_world_method,
                 G_TYPE_STRING,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);

  // creating some custom infos
  data::Tree::ptr tree = data::Tree::make();
  tree->graft(".child1.child2", data::Tree::make("switch"));
  tree->graft(".child1.child3", data::Tree::make(1.2f));
  tree->graft(".child1.child2.bla1", data::Tree::make("wire"));
  tree->graft(".child1.child2.bla2", data::Tree::make("hub"));
  // attaching it to the quiddity (at the root)
  graft_tree(".custom.information.", tree);
  g_debug("hello from plugin");
  return true;
}

gchar *DummyPlugin::my_hello_world_method(gchar *first_arg, void *user_data) {
  DummyPlugin *context = static_cast<DummyPlugin *>(user_data);
  g_debug("hello world from myplugin");
  context->hello_ = std::string("hello ") + first_arg;
  // the g_free will be invoked by the method system:
  return g_strdup(context->hello_.c_str());  
}

// bool DummyPlugin::start() {
//   g_debug("start from my plugin");
//   return true;
// }

// bool DummyPlugin::stop() {
//   g_debug("stop from my plugin");
//   return true;
// }

}  // namespace switcher
