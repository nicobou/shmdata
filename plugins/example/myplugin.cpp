/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
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
				       "GPL",
				       "myplugin",				
				       "Nicolas Bouillot");

  bool
  MyPlugin::init ()
  {
    custom_props_.reset (new CustomPropertyHelper ());
    
    myprop_ = false;
    
    myprop_prop_ = 
      custom_props_->make_boolean_property ("myprop", 
					    "myprop is a boolean property",
					    (gboolean)FALSE,
					    (GParamFlags) G_PARAM_READWRITE,
					    MyPlugin::set_myprop,
     					    MyPlugin::get_myprop,
					    this);

    register_property_by_pspec (custom_props_->get_gobject (), 
     				myprop_prop_, 
     				"myprop",
				"My Property");

    srand(time(0));
    set_name (g_strdup_printf ("myplugin%d",rand() % 1024));
    
    //g_print ("coucou from plugin\n");
    return true;
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
  }


}
