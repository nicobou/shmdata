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
#include "switcher/base-entity-life-manager.h"


namespace switcher
{

  BaseEntity::BaseEntity ()
  {}

  
  BaseEntity::~BaseEntity () { 
    g_print ("call: BaseEntity destructor for %s\n",get_name().c_str());
    //TODO remove properties & methods
  }

  std::string
  BaseEntity::get_name()
  {
    return std::string (name_);
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
  
  //return -1 if method not found
  int 
  BaseEntity::method_get_num_value_args (std::string function_name)
  {
    if (methods_.find( function_name ) == methods_.end())
      {
	g_printerr ("BaseEntity::method_get_num_value_args error: method %s not found\n",function_name.c_str());
	return -1;
      }
    else 
      return (int)methods_[function_name]->get_num_of_value_args(); 
  }

  //return -1 if method not found
  int 
  BaseEntity::method_get_num_pointer_args (std::string function_name)
  {
    if (methods_.find( function_name ) == methods_.end())
      {
	g_printerr ("BaseEntity::method_get_num_value_args error: method %s not found\n",function_name.c_str());
	return -1;
      }
    else 
      return (int)methods_[function_name]->get_num_of_pointer_args(); 
  }

  bool 
  BaseEntity::invoke_method (std::string function_name, 
			     std::vector<std::string> args, 
			     std::vector<void *> pointers)
  {
    if (methods_.find( function_name ) == methods_.end())
      {
	g_printerr ("BaseEntity::invoke_method error: method %s not found\n",function_name.c_str());
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

  bool 
  BaseEntity::set_method_description (std::string method_name,
				      std::string short_description,
				      std::vector<std::pair<std::string,std::string> > arg_description)
  {
    
    if (methods_.find( method_name ) == methods_.end())
      {
	g_printerr ("cannot set description of not existing ");
	return false;
      }

    methods_[method_name]->set_description (method_name, short_description, arg_description);
    
    return true;
  }

    std::string 
  BaseEntity::get_methods_description ()
  {
    std::string res;
    res.append ("{ methods: [ \n");
    for(std::map<std::string, Method::ptr>::iterator it = methods_.begin(); it != methods_.end(); ++it) 
      {
	if (it != methods_.begin())
	  res.append (", \n");
	res.append (it->second->get_description ());
      }
    res.append ("\n ]}");
    return res;
  }

  std::string 
  BaseEntity::get_method_description (std::string method_name)
  {
    if (methods_.find( method_name ) == methods_.end())
      return "";
    
    Method::ptr meth = methods_[method_name];
    return meth->get_description ();
  }

  

  std::string 
  BaseEntity::get_properties_description ()
  {
    std::string res;
    res.append ("{ properties: [ \n");
    for(std::map<std::string, Property::ptr>::iterator it = properties_.begin(); it != properties_.end(); ++it) 
      {
	if (it != properties_.begin())
	  res.append (", \n");

	res.append ("{ \"name\": \"");
	res.append (it->first);
	res.append ("\", ");
        res.append (" \"description\": ");
	res.append (it->second->get_description ());
	res.append ("}");
      }
    res.append (" ]}");
    return res;
  }

  std::string 
  BaseEntity::get_property_description (std::string name)
  {
    if (properties_.find( name ) == properties_.end())
      return "";
    
    Property::ptr prop = properties_[name];
    return prop->get_description ();
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

  void 
   BaseEntity::set_life_manager (std::tr1::shared_ptr<BaseEntityLifeManager> life_manager)
  {
    life_manager_ = life_manager; 
  }
 
}
