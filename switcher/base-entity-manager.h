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
 * The BaseEntityManager class
 */

#ifndef __SWITCHER_BASE_ENTITY_MANAGER_H__
#define __SWITCHER_BASE_ENTITY_MANAGER_H__

#include <vector>
#include <map>
#include <string>
#include "switcher/base-entity-life-manager.h"

 namespace switcher 
 { 

   class BaseEntityManager 
   { 
   public: 
     typedef std::tr1::shared_ptr<BaseEntityManager> ptr; 
    
     BaseEntityManager(); 
     ~BaseEntityManager(); 

     //properties
     std::string get_properties_description (std::string entity_name); //json formated
     bool set_property (std::string entity_name,
			std::string property_name,
			std::string property_value);
     
     std::string get_property (std::string entity_name, 
			       std::string property_name);
     
     //methods 
     std::string get_methods_description (std::string entity_name); //json formated
     bool invoke (std::string entity_name, 
		  std::string method_name,
		  std::vector<std::string> args);  

     //life manager
     std::vector<std::string> get_classes (); //know what entity can be created
     std::vector<std::string> get_entities (); //know instances
     std::string create (std::string entity_class_name); //returns the name
     bool remove (std::string entity_name);
     BaseEntityLifeManager::ptr get_life_manager ();

   private: 
     BaseEntityLifeManager::ptr life_manager_; //may be shared with others for automatic entity creation 

   }; 

 } // end of namespace 

#endif  




