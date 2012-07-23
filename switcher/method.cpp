/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

/**
 * The Method class
 */

#include "switcher/method.h"
namespace switcher
{

  Method::Method () :
    closure_ (NULL)
  {}

  Method::~Method ()
  {
    if (closure_ != NULL)
      g_closure_unref (closure_);
  }
   

  void 
  Method::set_method (void *method, std::vector<GType> arg_types, gpointer user_data)  
  {
    if (method == NULL)
      {
	g_printerr ("Method::set_method is call with a NULL function\n");
	return;
      }
    closure_ = g_cclosure_new (G_CALLBACK (method), user_data, Method::destroy_data);
    g_closure_set_marshal  (closure_,g_cclosure_marshal_generic);
    
    arg_types_ = arg_types;
}


  bool 
  Method::invoke(std::vector<std::string> args)
  {
    if (args.size () != arg_types_.size())
      {
	g_printerr ("Method::invoke number of arguments does not correspond to the size of argument types\n");
	return false;
      }

    GValue params[arg_types_.size()];
    
    for (guint i=0; i <args.size(); i++)
      {
	params[i] = G_VALUE_INIT;
	g_value_init (&params[i],arg_types_[i]);
	if ( !gst_value_deserialize (&params[i],args[i].c_str()))
	  {
	    g_printerr ("Method::invoke string not transformable into gvalue (argument: %s) \n",
			args[i].c_str());
	  }
      }
    
    GValue result_value = G_VALUE_INIT;
    gboolean result;
    g_value_init (&result_value, G_TYPE_BOOLEAN);
    
    g_closure_invoke (closure_, &result_value, arg_types_.size(), params, NULL); 
    result = g_value_get_boolean (&result_value);

    //unset
    g_value_unset (&result_value);
    for (guint i=0; i < arg_types_.size(); i++)
	g_value_unset (&params[i]);

    return result;
  } 
  
  void
  Method::destroy_data (gpointer  data,
			GClosure *closure)
  {
    g_print ("Method::destroy data\n");
  }
  
}
