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

#include "switcher/base-entity-manager.h"
#include "switcher/base-entity.h" 

 namespace switcher
 {

   BaseEntityManager::BaseEntityManager()
    {
      life_manager_.reset (new BaseEntityLifeManager());
    }

    BaseEntityManager::~BaseEntityManager()
    {
      g_print ("base entity manager destructed\n");
    }
  
   std::vector<std::string> 
   BaseEntityManager::get_properties (std::string entity_name)
   {
     return (life_manager_->get (entity_name))->get_property_names ();
   }

   bool
   BaseEntityManager::set_property (std::string entity_name,
				    std::string property_name,
				    std::string property_value)
   {
     return (life_manager_->get (entity_name))->set_property(property_name.c_str(),property_value.c_str());
   }

   std::string
   BaseEntityManager::get_property (std::string entity_name,
				    std::string property_name)
   {
     return (life_manager_->get (entity_name))->get_property(property_name.c_str());
   }

   std::vector<std::string>
   BaseEntityManager::get_methods (std::string entity_name)
   {
     return (life_manager_->get (entity_name))->get_method_names ();
   }

   bool 
   BaseEntityManager::invoke (std::string entity_name, 
			      std::string function_name,
			      std::vector<std::string> args)
   {
     g_print ("   BaseEntityManager::entity_invoke_method %s %s, arg size %d\n",entity_name.c_str(), function_name.c_str(), args.size ());
     
     if (!life_manager_->exists (entity_name))
       {
	 g_printerr ("entity %s not found, cannot invoke\n",entity_name.c_str());
	 return false;
       }
     BaseEntity::ptr entity = life_manager_->get (entity_name);

     int num_val = entity->method_get_num_value_args(function_name);

     if (num_val == -1) 
       {
	 g_printerr ("function %s not found, cannot invoke\n",function_name.c_str());
	 return false;
       }

     int num_pointer = entity->method_get_num_pointer_args(function_name);

     if ((int)args.size () != num_val + num_pointer)
       {
	 g_printerr ("invoking %s/%s, number of arguments does not match\n",entity_name.c_str(),function_name.c_str());
	 return false;
       }
     
     //checking if pointer to entity must be retrieved     
     if ((int)args.size() == num_val)
       //invoke with value only
       return entity->invoke_method (function_name, args);
     else
       {
	 //invoke with pointer to entity
	 std::vector<std::string> value_args (args.begin(), args.begin() + num_val);
	 std::vector<void *> entity_args;
	 
	 for(std::vector<std::string>::iterator it = args.begin() + num_val; it != args.end(); ++it) 
	   {
	     if (!life_manager_->exists (*it))
	       {
		 g_printerr ("BaseEntityManager::entity_invoke_method error: entity %s not found\n",
			     (*it).c_str());
		 return false;
	       }
	     else
	       {
		 BaseEntity::ptr retrieved_entity = life_manager_->get (*it);//entities_.lookup (*it);
		 entity_args.push_back ((void *)retrieved_entity.get());
	       }
	   }
	 bool res = entity->invoke_method (function_name, value_args,entity_args);
	 return res;
       }
   } 
   
   std::string
   BaseEntityManager::create (std::string entity_class)
   {
     if(!life_manager_->class_exists (entity_class))
       return "";
     
     BaseEntity::ptr entity = life_manager_->create (entity_class);
     //give reference to life manager to entity
     entity->set_life_manager (get_life_manager());
     return entity->get_name ();
   }


   bool
   BaseEntityManager::remove (std::string entity_name)
   {
     return life_manager_->remove (entity_name);
   }

       std::vector<std::string> 
    BaseEntityManager::get_classes ()
    {
      return life_manager_->get_classes ();
    }

    std::vector<std::string> 
    BaseEntityManager::get_entities ()
    {
      return life_manager_->get_instances ();
    }

   BaseEntityLifeManager::ptr 
   BaseEntityManager::get_life_manager ()
   {
     // BaseEntityLifeManager::ptr life_manager_copy = life_manager_;
     // return life_manager_copy;
     return life_manager_;
   }

 }
