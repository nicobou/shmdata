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
 * The QuiddityManager class
 */

#ifndef __SWITCHER_QUIDDITY_MANAGER_H__
#define __SWITCHER_QUIDDITY_MANAGER_H__

#include <vector>
#include <map>
#include <string>
#include "switcher/quiddity-life-manager.h"

 namespace switcher 
 { 

   class QuiddityManager 
   { 
   public: 
     typedef std::tr1::shared_ptr<QuiddityManager> ptr; 
    
     QuiddityManager();//will get name "default"
     QuiddityManager(std::string name); 
     ~QuiddityManager(); 

     //properties
     std::string get_properties_description (std::string quiddity_name); //json formated
     std::string get_property_description (std::string quiddity_name, std::string property_name); //json formated
     bool set_property (std::string quiddity_name,
			std::string property_name,
			std::string property_value);
     
     std::string get_property (std::string quiddity_name, 
			       std::string property_name);
     
     //methods 
     std::string get_methods_description (std::string quiddity_name); //json formated
     std::string get_method_description (std::string quiddity_name, std::string method_name); //json formated
     bool invoke (std::string quiddity_name, 
		  std::string method_name,
		  std::vector<std::string> args);  

     //life manager
     std::vector<std::string> get_classes (); //know which quiddities can be created
     std::vector<std::string> get_quiddities (); //know instances
     std::string create (std::string class_name); //returns the name
     std::string create (std::string class_name, 
			 std::string nick_name);
     bool remove (std::string quiddity_name);
     QuiddityLifeManager::ptr get_life_manager ();

   private: 
     QuiddityLifeManager::ptr life_manager_; //may be shared with others for automatic quiddity creation 
     std::string name_;
   }; 

 } // end of namespace 

#endif  




