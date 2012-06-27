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



 namespace switcher 
 { 

   class BaseEntity;

   class BaseEntityManager 
   { 
   public: 
     typedef std::tr1::shared_ptr<BaseEntityManager> ptr; 
    
     BaseEntityManager(); 
     ~BaseEntityManager(); 

     std::vector<std::string> get_list_of_creatable_entities (); 
     
     std::vector<std::string> get_list_of_entities (); 
 

     //properties

     bool set_entity_property (std::string entity_name,
			       std::string property_name,
			       std::string property_value);

     std::string get_entity_property (std::string entity_name, 
				      std::string property_name);

     //create entity and insert it into the entity set
     std::tr1::shared_ptr<BaseEntity>  
       create_entity (std::string entity_class_name); 
 
     //should be called from the entity destructor
     void unref_entity (std::string entity_name);
     
   private: 
     Factory<BaseEntity, std::string> abstract_factory_;
     //do not use shared pointers here since inserting and erasing 
     //are done during the entity creation and destruction 
     std::map<std::string, BaseEntity *> entities_;

   }; 


 } // end of namespace 

#endif  




