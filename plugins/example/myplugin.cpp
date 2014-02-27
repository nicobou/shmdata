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

#include "myplugin.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(MyPlugin,
				       "My Plugin",
				       "test", 
				       "Creates a quiddity from a plugin",
				       "LGPL",
				       "myplugin",				
				       "Nicolas Bouillot");
  MyPlugin::MyPlugin () :
    custom_props_ (new CustomPropertyHelper ()),
    myprop_ (false),
    myprop_prop_ (NULL),
    hello_ (g_strdup ("hello"))
  {}

  bool
  MyPlugin::init ()
  {
    init_startable (this);
    myprop_prop_ = 
      custom_props_->make_boolean_property ("myprop", //name 
					    "myprop is a boolean property", //description
					    (gboolean)FALSE, //default value
					    (GParamFlags) G_PARAM_READWRITE,  
					    MyPlugin::set_myprop,
     					    MyPlugin::get_myprop,
					    this);
    install_property_by_pspec (custom_props_->get_gobject (), 
     				myprop_prop_, 
     				"myprop",
				"My Property"); //long name

    install_method ("Hello World", //long name
		    "hello-world", //name
		    "say hello and repeat first argument", //description
		    "the hello answer", //return description
		    Method::make_arg_description ("Text To Repeat", //first arg long name
						  "text", //fisrt arg name
						  "string", //first arg description
						  NULL),
  		    (Method::method_ptr) &my_hello_world_method, 
		    G_TYPE_STRING,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    this);

    g_debug ("hello from plugin");
    return true;
  }
  
  MyPlugin::~MyPlugin ()
  {
    g_free (hello_);
  }
  
  gboolean 
  MyPlugin::get_myprop (void *user_data)
  {
    MyPlugin *context = static_cast<MyPlugin *> (user_data);
    return context->myprop_;
  }

  void 
  MyPlugin::set_myprop (gboolean myprop, void *user_data)
  {
    MyPlugin *context = static_cast<MyPlugin *> (user_data);
    context->myprop_ = myprop;
    GObjectWrapper::notify_property_changed (context->gobject_->get_gobject (), 
					     context->myprop_prop_);
  }


  gchar *
  MyPlugin::my_hello_world_method (gchar *first_arg, void *user_data)
  {
    MyPlugin *context = static_cast<MyPlugin *> (user_data);
    
    g_debug ("hello world from myplugin");
    g_free (context->hello_);
    context->hello_ = g_strdup_printf ("hello %s",first_arg);
    return context->hello_;
  }

  bool
  MyPlugin::start ()
  {
    g_debug ("start from my plugin");
    return true;
  }

  bool
  MyPlugin::stop ()
  {
    g_debug ("stop from my plugin");
    return true;
  }
}
