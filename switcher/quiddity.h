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
 * The Quiddity class
 */


#ifndef __SWITCHER_QUIDDITY_H__
#define __SWITCHER_QUIDDITY_H__

#include <string>
#include <vector>
#include <tr1/memory>
#include <map>
#include <gst/gst.h>
#include "switcher/property.h"
#include "switcher/method.h"
#include "switcher/quiddity-documentation.h"
#include "switcher/quiddity-life-manager.h"


namespace switcher
{
  class QuiddityLifeManager;
 
  class Quiddity
  {
  public:
    typedef std::tr1::shared_ptr<Quiddity> ptr;
    
    Quiddity ();
    virtual ~Quiddity ();
    
    //class documentation
    //virtual QuiddityDocumentation get_documentation() = 0;
    
    //instance name
    std::string get_name ();
    
    //properties
    std::string get_property_description (std::string property_name);
    std::string get_properties_description ();
    bool set_property (std::string name, 
		       std::string value);
    std::string get_property (std::string name);
    
    //methods
    std::string get_method_description (std::string method_name);
    std::string get_methods_description ();
    bool invoke_method (std::string function_name,
			std::vector<std::string> args);
    bool invoke_method (std::string function_name,
			std::vector<std::string> args,
			std::vector<void *> pointers);
    int method_get_num_value_args (std::string function_name); //returns -1 if method not found
    int method_get_num_pointer_args (std::string function_name); //returns -1 if method not found
    
  private:
    //properties are registered by derived class
    std::map<std::string, Property::ptr> properties_;
    std::map<std::string, Method::ptr> methods_;

  protected:
    std::string name_;
    //property name will be <prefix>/<object_property>
    bool register_property (GObject *object, 
			    std::string object_property, 
			    std::string prefix);
    bool register_method (std::string method_name,
			  void *method, 
			  std::vector<GType> arg_types, 
			  gpointer user_data);
    bool set_method_description (std::string method_name,
				 std::string short_description,
				 std::vector<std::pair<std::string,std::string> > arg_description);
    //use a consistent naming for shmdatas
    std::string make_shmdata_writer_name (std::string suffix);

    //used in order to dynamically create other quiddity, weak_ptr is used in order to 
    //avoid circular references to the life manager 
    std::tr1::weak_ptr<QuiddityLifeManager> life_manager_;

    
  };
  
} // end of namespace

#endif // ifndef
