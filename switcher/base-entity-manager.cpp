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
#include "switcher/video-test-source.h"
#include "switcher/ctrl-server.h"
#include "switcher/runtime.h"


 namespace switcher
 {

   BaseEntityManager::BaseEntityManager()
    {
      //registering base entity to make available
      abstract_factory_.register_class<Runtime> ("runtime");
      abstract_factory_.register_class<VideoTestSource> ("videotestsource");
      abstract_factory_.register_class<CtrlServer> ("controlserver");

    }

    BaseEntityManager::~BaseEntityManager()
    {
      g_print ("base entity manager destructed\n");
    }
  
    std::vector<std::string> 
    BaseEntityManager::get_list_of_entity_classes ()
    {
      return abstract_factory_.get_keys ();
    }

    std::vector<std::string> 
    BaseEntityManager::get_list_of_entities ()
    {
      return entities_.get_keys();
    }
   
   std::vector<std::string> 
   BaseEntityManager::get_property_names (std::string entity_name)
   {
     return (entities_.lookup (entity_name))->get_property_names ();
   }

   bool
   BaseEntityManager::set_entity_property (std::string entity_name,
			       std::string property_name,
			       std::string property_value)
   {
     return (entities_.lookup (entity_name))->set_property(property_name.c_str(),property_value.c_str());
     //(*hashed_->lookup (entity_name))->set_property(property_name.c_str(),property_value.c_str());
     //return true;
   }

   std::string
   BaseEntityManager::get_entity_property (std::string entity_name,
					   std::string property_name)
   {
     return (entities_.lookup (entity_name))->get_property(property_name.c_str());
     //return (*hashed_->lookup (entity_name))->get_property(property_name.c_str());
   }

   std::vector<std::string>
   BaseEntityManager::get_list_of_method_names(std::string entity_name)
   {
     return (entities_.lookup (entity_name))->get_method_names ();
     //return (*hashed_->lookup (entity_name))->get_method_names ();
   }

   bool 
   BaseEntityManager::entity_invoke_method (std::string entity_name, 
					    std::string function_name,
					    std::vector<std::string> args)
   {
     g_print ("   BaseEntityManager::entity_invoke_method %s %s\n",entity_name.c_str(), function_name.c_str());
     
     //BaseEntity::ptr entity = (*hashed_->lookup (entity_name));
     BaseEntity::ptr entity = entities_.lookup (entity_name);

     int num_val = entity->method_get_num_value_args(function_name);

     if (num_val == -1) 
       {
	 g_printerr ("function %s not found, cannot invoke\n",entity_name.c_str());
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
	     //BaseEntity::ptr *retrieved_entity = hashed_->lookup (*it);
	     if (!entities_.contains (*it))
	       {
		 g_printerr ("BaseEntityManager::entity_invoke_method error: entity %s not found\n",
			     (*it).c_str());
		 return false;
	       }
	     else
	       {
		 BaseEntity::ptr retrieved_entity = entities_.lookup (*it);
		 entity_args.push_back ((void *)retrieved_entity.get());
	       }
	   }
	 bool res = entity->invoke_method (function_name, value_args,entity_args);
	 return res;
       }
   } 
   
   BaseEntity::ptr
   BaseEntityManager::create_entity (std::string entity_class)
   {
     //std::cout << entity_class << std::endl;
     BaseEntity::ptr entity = abstract_factory_.create (entity_class);

     g_print ("create_entity %p %p\n",&entity,entity.get());

     if (entity.get() != NULL)
       {
	 //hashed_->insert (entity->get_name(),&entity);
	 entities_.insert (entity->get_name(),entity);
       }
     
     return entity;
   }


   bool
   BaseEntityManager::delete_entity (std::string entity_name)
   {
     //return (hashed_->remove (entity_name));
     return entities_.remove (entity_name);
   }


 }
