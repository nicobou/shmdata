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

/**
 * The Quiddity class
 */


#ifndef __SWITCHER_QUIDDITY_H__
#define __SWITCHER_QUIDDITY_H__

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <gst/gst.h>
#include "property.h"
#include "method.h"
#include "quiddity-documentation.h"
#include "quiddity-life-manager.h"
#include "json-builder.h"

namespace switcher
{
  class QuiddityLifeManager;
 
  class Quiddity
  {
  public:
    typedef std::shared_ptr<Quiddity> ptr;
    
    Quiddity ();
    virtual ~Quiddity ();
    
    //class documentation
    virtual QuiddityDocumentation get_documentation() = 0;
    
    //class initialisation
    virtual bool init () = 0;

    //instance name
    std::string get_name ();
    std::string get_nick_name ();
    bool set_nick_name (std::string nick_name);

    //properties
    std::string get_property_description (std::string property_name);
    std::string get_properties_description ();
    bool set_property (std::string name, 
		       std::string value);
    std::string get_property (std::string name);
    bool subscribe_property (std::string name,
			     Property::Callback cb, 
			     void *user_data);
    bool unsubscribe_property (std::string name,
			       Property::Callback cb,
			       void *user_data);

    
    //methods
    std::string get_method_description (std::string method_name);
    std::string get_methods_description ();
    bool invoke_method (std::string function_name,
			std::vector<std::string> args);
    int method_get_num_value_args (std::string function_name); //returns -1 if method not found
    int method_get_num_pointer_args (std::string function_name); //returns -1 if method not found
    
    //shmdata socket names
    static std::string get_socket_name_prefix ();
    static std::string get_socket_dir ();

    //life manager  initialization
    void set_life_manager (std::shared_ptr<QuiddityLifeManager> life_manager);

  private:
    //properties are registered by derived class
    std::map<std::string, Property::ptr> properties_;
    JSONBuilder::ptr properties_description_;
    std::map<std::string, Method::ptr> methods_;
    JSONBuilder::ptr methods_description_;
    std::string name_;
    std::string nick_name_;
 
  protected:
    //naming
    bool set_name (std::string name);

    //property name will be <prefix>/<object_property>
    bool register_property (GObject *object, 
			    std::string gobject_property_name, 
			    std::string name_to_give);
    bool register_property_by_pspec (GObject *object, 
				     GParamSpec *pspec, 
				     std::string name_to_give);
    bool register_method (std::string method_name,
			  void *method, 
			  Method::args_types arg_types, 
			  gpointer user_data);
    bool set_method_description (const std::string method_name,
				 const std::string short_description,
				 const Method::args_doc arg_description);
    //use a consistent naming for shmdatas FIXME move that to segment
    std::string make_file_name (std::string suffix);

    //used in order to dynamically create other quiddity, weak_ptr is used in order to 
    //avoid circular references to the life manager 
    std::weak_ptr<QuiddityLifeManager> life_manager_;
  };
  
} // end of namespace

#endif // ifndef
