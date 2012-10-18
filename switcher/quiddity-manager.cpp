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

  bool QuiddityManager::gmainloop_initialized_ = false;

  QuiddityManager::QuiddityManager() :
    name_ ("default")
  {
    init_gmainloop ();
    init_command_sync ();
    life_manager_.reset (new QuiddityLifeManager());
  }

  QuiddityManager::QuiddityManager(std::string name) :
    name_ (name)
  {
    init_gmainloop ();
    init_command_sync ();
    life_manager_.reset (new QuiddityLifeManager(name));
  }

  QuiddityManager::~QuiddityManager()
  {
    clear_command_sync ();
    //FIXME count instances and do quit main loop
    //g_main_loop_quit (mainloop_);
    g_print ("base quiddity manager destructed\n");
  }

  void
  QuiddityManager::init_gmainloop ()
  {
    if (!gmainloop_initialized_)
      {
	gst_init (NULL,NULL);
	mainloop_ = g_main_loop_new (NULL, FALSE);
	thread_ = g_thread_new ("SwitcherMainLoop", GThreadFunc(main_loop_thread), this);
	gmainloop_initialized_ = true;
      }
  }
  
  gpointer
  QuiddityManager::main_loop_thread (gpointer user_data)
  {
    QuiddityManager *context = static_cast<QuiddityManager*>(user_data);
    g_main_loop_run (context->mainloop_);
    return NULL;
  }

  void
  QuiddityManager::init_command_sync ()
  {
    exec_cond_ = g_cond_new ();
    exec_mutex_ = g_mutex_new ();
  }

  void
  QuiddityManager::clear_command_sync ()
  {
    //FIXME may get the mutex in order to clean quit
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
    //preparing the command
    command_.make (QuiddityCommand::create,quiddity_class);
    invoke_in_gmainloop ();
    return command_.exec_return_[0];
    //    return life_manager_->create (quiddity_class, life_manager_);
  }

  std::string
  QuiddityManager::create (std::string quiddity_class, std::string nick_name)
  {
    return life_manager_->create (quiddity_class, nick_name, life_manager_);
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
  
  void
  QuiddityManager::invoke_in_gmainloop ()
  {
    g_idle_add ((GSourceFunc) QuiddityManager::gmainloop_run, (gpointer) this);
    g_mutex_lock (exec_mutex_);
    g_cond_wait (exec_cond_, exec_mutex_);
    g_mutex_unlock (exec_mutex_);
  }

  gboolean 
  QuiddityManager::gmainloop_run (gpointer user_data)
  {
    QuiddityManager *context = static_cast<QuiddityManager *>(user_data);

    g_mutex_lock (context->exec_mutex_);
    switch (context->command_.name_)
      {
         case QuiddityCommand::get_classes:
            break;
         case QuiddityCommand::get_quiddities:
            break;
         case QuiddityCommand::create:
	   context->command_.exec_return_.push_back(context->life_manager_->create (context->command_.args_[0], context->life_manager_));
          break;
         case QuiddityCommand::create_nick_named:
            break;
         case QuiddityCommand::remove:
            break;
         case QuiddityCommand::get_properties_description:
            break;
         case QuiddityCommand::get_property_description:
            break;
         case QuiddityCommand::set_property:
            break;
         case QuiddityCommand::get_property:
            break;
         case QuiddityCommand::get_methods_description:
            break;
         case QuiddityCommand::get_method_description:
            break;
         case QuiddityCommand::invoke:
            break;
         default:
	   g_printerr("unknown command");
      }
    g_cond_signal (context->exec_cond_);
    g_mutex_unlock (context->exec_mutex_);
    return FALSE; //this thread should be removed from the main loop
  }
  
  }
