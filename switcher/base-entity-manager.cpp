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
      abstract_factory_.Register<Runtime> ("runtime");
      abstract_factory_.Register<VideoTestSource> ("videotestsource");
      abstract_factory_.Register<CtrlServer> ("controlserver");
    }

    BaseEntityManager::~BaseEntityManager()
    {
      g_print ("base entity destructed\n");
      //TODO remove reference to this in the entities
    }
  
    std::vector<std::string> 
    BaseEntityManager::get_list_of_entity_classes ()
    {
      return abstract_factory_.getList ();
    }

    std::vector<std::string> 
    BaseEntityManager::get_list_of_entities ()
    {
      std::vector<std::string> list_of_entities;
      for(std::map<std::string, std::tr1::shared_ptr<BaseEntity> >::iterator it = entities_.begin(); it != entities_.end(); ++it) {
	list_of_entities.push_back(it->first);
      }
      return list_of_entities;
    }
   
   std::vector<std::string> 
   BaseEntityManager::get_property_names (std::string entity_name)
   {
     return entities_[entity_name]->get_property_names ();
   }

   bool
   BaseEntityManager::set_entity_property (std::string entity_name,
			       std::string property_name,
			       std::string property_value)
   {
     entities_[entity_name]->set_property(property_name.c_str(),property_value.c_str());
     return true;
   }

   std::string
   BaseEntityManager::get_entity_property (std::string entity_name,
					   std::string property_name)
   {
     return entities_[entity_name]->get_property(property_name.c_str());
   }

   std::vector<std::string>
   BaseEntityManager::get_list_of_method_names(std::string entity_name)
   {
     return entities_[entity_name]->get_method_names ();
   }

   bool 
   BaseEntityManager::entity_invoke_method (std::string entity_name, 
					    std::string function_name,
					    std::vector<std::string> args)
   {
     g_print ("   BaseEntityManager::entity_invoke_method\n");
     return entities_[entity_name]->invoke_method (function_name, args);
   } 
   
   bool 
   BaseEntityManager::entity_invoke_method_with_name_args (std::string entity_name, 
							   std::string function_name,
							   std::vector<std::string> args,
							   std::vector<std::string> entity_names_args)
   {
     std::vector<void *> entity_args;
     for(std::vector<std::string>::iterator it = entity_names_args.begin(); it != entity_names_args.end(); ++it) 
       {
	 std::map<std::string,BaseEntity::ptr>::iterator ent = entities_.find(*it);
	 if (ent == entities_.end())
	   {
	     g_printerr ("BaseEntityManager::entity_invoke_method_with_name_args error: entity %s not found\n",
			 (*it).c_str());
	     return false;
	   }
	 entity_args.push_back((void *)&ent->second);
       }
     
     return entities_[entity_name]->invoke_method (function_name, args,entity_args);
   } 
     

    std::tr1::shared_ptr<BaseEntity>
    BaseEntityManager::create_entity (std::string entity_class)
    {
      //std::cout << entity_class << std::endl;
      std::tr1::shared_ptr<BaseEntity> entity = abstract_factory_.Create (entity_class);
      
      if (entity.get() != NULL)
	{
	  entities_[entity->get_name()] = entity;
	}
      return entity;
    }

   

   bool
   BaseEntityManager::delete_entity (std::string entity_name)
   {
     std::map<std::string, BaseEntity::ptr >::iterator it;
     it = entities_.find(entity_name);
     if (it != entities_.end())
       {
	 entities_.erase(entity_name);
	 return true;
       }
     else
       return false;
   }


 }
