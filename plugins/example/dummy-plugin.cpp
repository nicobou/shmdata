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

DummyPlugin::DummyPlugin(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

bool DummyPlugin::init() {
  init_startable(this);
  myprop_prop_ = custom_props_->make_boolean_property("myprop",       // name
                                                      "myprop is a boolean property",  // description
                                                      (gboolean) FALSE,  // default value
                                                      (GParamFlags)
                                                      G_PARAM_READWRITE,
                                                      DummyPlugin::set_myprop,
                                                      DummyPlugin::get_myprop,
                                                      this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            myprop_prop_,
                            "myprop",
                            "My Property");  // long name

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

gboolean DummyPlugin::get_myprop(void *user_data) {
  DummyPlugin *context = static_cast<DummyPlugin *>(user_data);
  return context->myprop_;
}

void DummyPlugin::set_myprop(gboolean myprop, void *user_data) {
  DummyPlugin *context = static_cast<DummyPlugin *>(user_data);
  context->myprop_ = myprop;
  GObjectWrapper::notify_property_changed(context->gobject_->get_gobject(),
                                          context->myprop_prop_);
}

gchar *DummyPlugin::my_hello_world_method(gchar *first_arg, void *user_data) {
  DummyPlugin *context = static_cast<DummyPlugin *>(user_data);
  g_debug("hello world from myplugin");
  context->hello_ = std::string("hello ") + first_arg;
  // the g_free will be invoked by the method system:
  return g_strdup(context->hello_.c_str());  
}

bool DummyPlugin::start() {
  g_debug("start from my plugin");
  return true;
}

bool DummyPlugin::stop() {
  g_debug("stop from my plugin");
  return true;
}
}
