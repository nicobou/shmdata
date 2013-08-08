/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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
#include <cstdlib>  // For srand() and rand()
#include <ctime>    // For time()

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(MyPlugin,
				       "My Plugin",
				       "test", 
				       "Creates a quiddity from a plugin",
				       "LGPL",
				       "myplugin",				
				       "Nicolas Bouillot");

  bool
  MyPlugin::init ()
  {
    custom_props_.reset (new CustomPropertyHelper ());

    hello_ = g_strdup ("hello");
    myprop_ = false;
    myprop_prop_ = 
      custom_props_->make_boolean_property ("myprop", //name 
					    "myprop is a boolean property", //description
					    (gboolean)FALSE, //default value
					    (GParamFlags) G_PARAM_READWRITE,  
					    MyPlugin::set_myprop,
     					    MyPlugin::get_myprop,
					    this);
    register_property_by_pspec (custom_props_->get_gobject (), 
     				myprop_prop_, 
     				"myprop",
				"My Property"); //long name


    GType types[] = {G_TYPE_STRING};
    register_signal_action ("hello_world",
			    (void *)MyPlugin::my_signal_action, 
			    G_TYPE_STRING,
			    1,
			    types,
			    this);
    set_signal_description ("Hello World",
			    "hello_world",
			    "a hello world test method ",
			    Signal::make_arg_description("Who To Say Hello To",
							 "who",
							 "string that will be repeated with hello",
							 NULL));

    srand(time(0));
    set_name (g_strdup_printf ("myplugin%d",rand() % 1024));
    
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
  MyPlugin::my_signal_action (void *, gchar *first_arg, void *user_data)
  {
    MyPlugin *context = static_cast<MyPlugin *> (user_data);

    g_debug ("hello world from myplugin");
    g_free (context->hello_);
    context->hello_ = g_strdup_printf ("hello %s",first_arg);
    return context->hello_;
  }

}
