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

#include "switcher/quiddity-manager.h"
#include "switcher/quiddity.h" 

namespace switcher
{

  QuiddityManager::QuiddityManager() :
    name_ ("default")
  {
    life_manager_.reset (new QuiddityLifeManager());
  }

  QuiddityManager::QuiddityManager(std::string name) :
    name_ (name)
  {
    life_manager_.reset (new QuiddityLifeManager(name));
  }

  QuiddityManager::~QuiddityManager()
  {
    g_print ("base quiddity manager destructed\n");
  }
  
  std::string 
  QuiddityManager::get_properties_description (std::string quiddity_name)
  {
    return life_manager_->get_properties_description (quiddity_name);
  }

  std::string 
  QuiddityManager::get_property_description (std::string quiddity_name, std::string property_name)
  {
    return life_manager_->get_property_description (quiddity_name, property_name);
  }

  bool
  QuiddityManager::set_property (std::string quiddity_name,
				 std::string property_name,
				 std::string property_value)
  {
    return life_manager_->set_property(quiddity_name, property_name,property_value);
  }

  std::string
  QuiddityManager::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    return life_manager_->get_property(quiddity_name, property_name);
  }

  bool 
  QuiddityManager::invoke (std::string quiddity_name, 
			   std::string function_name,
			   std::vector<std::string> args)
  {
    return life_manager_->invoke (quiddity_name, function_name, args);
  } 

  std::string
  QuiddityManager::get_methods_description (std::string quiddity_name)
  {
    return life_manager_->get_methods_description (quiddity_name);
  }

  std::string
  QuiddityManager::get_method_description (std::string quiddity_name, std::string method_name)
  {
    return life_manager_->get_method_description (quiddity_name, method_name);
  }
   
  std::string
  QuiddityManager::create (std::string quiddity_class)
  {
    return life_manager_->create (quiddity_class, get_life_manager ());
  }

  std::string
  QuiddityManager::create (std::string quiddity_class, std::string nick_name)
  {
    return life_manager_->create (quiddity_class, nick_name, get_life_manager ());
  }

  bool
  QuiddityManager::remove (std::string quiddity_name)
  {
    return life_manager_->remove (quiddity_name);
  }

  std::vector<std::string> 
  QuiddityManager::get_classes ()
  {
    return life_manager_->get_classes ();
  }
   
  std::vector<std::string> 
  QuiddityManager::get_quiddities ()
  {
    return life_manager_->get_instances ();
  }

  QuiddityLifeManager::ptr 
  QuiddityManager::get_life_manager ()
  {
    return life_manager_;
  }

}
