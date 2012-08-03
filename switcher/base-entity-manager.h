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
#include "switcher/base-entity.h" 
#include "switcher/creator.h" 
#include "switcher/hash-table.h"


 namespace switcher 
 { 

   class BaseEntity;

   class BaseEntityManager 
   { 
   public: 
     typedef std::tr1::shared_ptr<BaseEntityManager> ptr; 
    
     BaseEntityManager(); 
     ~BaseEntityManager(); 

     //know what entity can be created
     std::vector<std::string> 
       get_list_of_entity_classes (); 
 
     //list entity instances 
     std::vector<std::string> 
       get_list_of_entities (); 
 
     //properties
     std::vector<std::string> get_property_names (std::string entity_name);
     bool set_entity_property (std::string entity_name,
			       std::string property_name,
			       std::string property_value);
     
     std::string get_entity_property (std::string entity_name, 
				      std::string property_name);
     
      //method 
     std::vector<std::string> get_list_of_method_names (std::string entity_name); 
     
     bool entity_invoke_method (std::string entity_name, 
				std::string function_name,
				std::vector<std::string> args);  

     bool entity_invoke_method_with_name_args (std::string entity_name, 
					       std::string function_name,
					       std::vector<std::string> args,
					       std::vector<std::string> entity_names_args);  

     //entity life cycle
     std::tr1::shared_ptr<BaseEntity>  
       create_entity (std::string entity_class_name); 
     bool
       delete_entity (std::string entity_name);


 
   private: 
     Factory<BaseEntity, std::string> abstract_factory_;
     std::map<std::string, BaseEntity::ptr > entities_;
     HashTable::ptr hashed_;
   }; 

 } // end of namespace 

#endif  




