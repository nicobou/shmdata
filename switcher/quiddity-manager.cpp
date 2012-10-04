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
      life_manager_.reset (new QuiddityLifeManager());
    }

    QuiddityManager::~QuiddityManager()
    {
      g_print ("base quiddity manager destructed\n");
    }
  
   std::string 
   QuiddityManager::get_properties_description (std::string quiddity_name)
   {

   if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot get description of properties\n",quiddity_name.c_str());
	 return "";
       }
     return (life_manager_->get (quiddity_name))->get_properties_description ();
   }

   std::string 
   QuiddityManager::get_property_description (std::string quiddity_name, std::string property_name)
   {

   if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot get description of properties\n",quiddity_name.c_str());
	 return "";
       }
     return (life_manager_->get (quiddity_name))->get_property_description (property_name);
   }

   bool
   QuiddityManager::set_property (std::string quiddity_name,
				    std::string property_name,
				    std::string property_value)
   {
     if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot set property\n",quiddity_name.c_str());
	 return false;
       }
     return (life_manager_->get (quiddity_name))->set_property(property_name.c_str(),property_value.c_str());
   }

   std::string
   QuiddityManager::get_property (std::string quiddity_name,
				    std::string property_name)
   {
     if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot get property\n",quiddity_name.c_str());
	 return "error, quiddity not found";
       }
     return (life_manager_->get (quiddity_name))->get_property(property_name.c_str());
   }

   // std::vector<std::string>
   // QuiddityManager::get_methods (std::string quiddity_name)
   // {
   //   return (life_manager_->get (quiddity_name))->get_method_names ();
   // }


   bool 
   QuiddityManager::invoke (std::string quiddity_name, 
			      std::string function_name,
			      std::vector<std::string> args)
   {
     //g_print ("   QuiddityManager::quiddity_invoke_method %s %s, arg size %d\n",quiddity_name.c_str(), function_name.c_str(), args.size ());
     
     if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot invoke\n",quiddity_name.c_str());
	 return false;
       }
     Quiddity::ptr quiddity = life_manager_->get (quiddity_name);

     int num_val = quiddity->method_get_num_value_args(function_name);

     if (num_val == -1) 
       {
	 g_printerr ("function %s not found, cannot invoke\n",function_name.c_str());
	 return false;
       }

     int num_pointer = quiddity->method_get_num_pointer_args(function_name);

     if ((int)args.size () != num_val + num_pointer)
       {
	 g_printerr ("invoking %s/%s, number of arguments does not match\n",quiddity_name.c_str(),function_name.c_str());
	 return false;
       }
     
     //checking if pointer to quiddity must be retrieved     
     if ((int)args.size() == num_val)
       //invoke with value only
       return quiddity->invoke_method (function_name, args);
     else
       {
	 //invoke with pointer to quiddity
	 std::vector<std::string> value_args (args.begin(), args.begin() + num_val);
	 std::vector<void *> quiddity_args;
	 
	 for(std::vector<std::string>::iterator it = args.begin() + num_val; it != args.end(); ++it) 
	   {
	     if (!life_manager_->exists (*it))
	       {
		 g_printerr ("QuiddityManager::quiddity_invoke_method error: quiddity %s not found\n",
			     (*it).c_str());
		 return false;
	       }
	     else
	       {
		 Quiddity::ptr retrieved_quiddity = life_manager_->get (*it);//quiddities_.lookup (*it);
		 quiddity_args.push_back ((void *)retrieved_quiddity.get());
	       }
	   }
	 bool res = quiddity->invoke_method (function_name, value_args,quiddity_args);
	 return res;
       }
   } 

   std::string
   QuiddityManager::get_methods_description (std::string quiddity_name)
   {
     if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot get description of methods\n",quiddity_name.c_str());
	 return "error, quiddity not found";
       }
     
     return (life_manager_->get (quiddity_name))->get_methods_description ();
   }

   std::string
   QuiddityManager::get_method_description (std::string quiddity_name, std::string method_name)
   {
     if (!life_manager_->exists (quiddity_name))
       {
	 g_printerr ("quiddity %s not found, cannot get description of methods\n",quiddity_name.c_str());
	 return "error, quiddity not found";
       }
     
     return (life_manager_->get (quiddity_name))->get_method_description (method_name);
   }
   
   std::string
   QuiddityManager::create (std::string quiddity_class)
   {
     if(!life_manager_->class_exists (quiddity_class))
       return "";
     
     Quiddity::ptr quiddity = life_manager_->create (quiddity_class);
     //give reference of life manager to the quiddity
     quiddity->set_life_manager (get_life_manager());
     return quiddity->get_name ();
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
     // QuiddityLifeManager::ptr life_manager_copy = life_manager_;
     // return life_manager_copy;
     return life_manager_;
   }

 }
