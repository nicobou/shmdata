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
 * The BaseEntity class
 */

#include "switcher/base-entity.h"

namespace switcher
{

  BaseEntity::BaseEntity ()
  {
    g_print ("call: BaseEntity constructor\n");
  }
  
  BaseEntity::~BaseEntity () { 
    g_print ("call: BaseEntity destructor for %s\n",get_name().c_str());
    //TODO remove properties & methods
  }

  std::string
  BaseEntity::get_name()
  {
    return name_;
  }

  
  bool
  BaseEntity::register_property (GObject *object, std::string object_property, std::string prefix)
  {
    GParamSpec *pspec = g_object_class_find_property (G_OBJECT_GET_CLASS(object), object_property.c_str());
    if (pspec == NULL)
      {
	g_printerr ("property not found %s\n",object_property.c_str());
	return false;
      }

    Property::ptr prop (new Property (object, pspec));

    std::string name ( prefix + "/" + object_property );
    if (properties_.find( name ) == properties_.end())
      {
	properties_[ name ] = prop; 
	return true;
      }
    else 
      {
	g_printerr ("registering name %s already exists\n",name.c_str());
	return false;
      }
  }
  
  bool 
  BaseEntity::invoke_method (std::string function_name, 
			     std::vector<std::string> args, 
			     std::vector<void *> pointers)
  {
    if (methods_.find( function_name ) == methods_.end())
      {
	g_printerr ("BaseEntity::invoke_method error: methode %s not found\n",function_name.c_str());
	return false;
      }
    else 
      {
	return methods_[function_name]->invoke (args, pointers); 
      }
 
  }

  bool 
  BaseEntity::invoke_method (std::string function_name, 
			     std::vector<std::string> args)
  {
    std::vector<void *> empty;
    return invoke_method (function_name, args, empty);
  }
  
  std::vector<std::string>
  BaseEntity::get_method_names ()
  {
    std::vector<std::string> list_of_methods;
    for(std::map<std::string, Method::ptr>::iterator it = methods_.begin(); it != methods_.end(); ++it) 
      {
	list_of_methods.push_back(it->first);
      }
    return list_of_methods;
  }


  bool
  BaseEntity::register_method (std::string method_name, void *method, std::vector<GType> arg_types, gpointer user_data)
  {
    if (method == NULL)
      {
	g_printerr ("fail registering %s (method is NULL)\n",method_name.c_str());
	return false;
      }
    
    Method::ptr meth (new Method ());
    meth->set_method (method, arg_types, user_data);

    if (methods_.find( method_name ) == methods_.end())
      {
	methods_[method_name] = meth;
	return true;
      }
    else 
      {
	g_printerr ("registering name %s already exists\n",method_name.c_str());
	return false;
      }

  }

  void
  BaseEntity::print_properties ()
  {
    for( std::map<std::string, Property::ptr>::iterator ii=properties_.begin(); ii!=properties_.end(); ++ii)
      {
     	g_print ("\n....\n%s\n",(*ii).first.c_str());
     	(*ii).second->print ();
      }
  }

  
  
  std::vector<std::string>
  BaseEntity::get_property_names ()
  {
    std::vector<std::string> list_of_properties;
    for(std::map<std::string, Property::ptr>::iterator it = properties_.begin(); it != properties_.end(); ++it) 
      {
	list_of_properties.push_back(it->first);
      }
    return list_of_properties;
  }

  bool 
  BaseEntity::set_property (std::string name, std::string value)
  {
    if (properties_.find( name ) == properties_.end())
      return false;

    Property::ptr prop = properties_[name];
    prop->set (value);
    return true;
  }

  std::string 
  BaseEntity::get_property (std::string name)
  {
    if (properties_.find( name ) == properties_.end())
      return "property not found";

    Property::ptr prop = properties_[name];
    return prop->get ();
  }

}
