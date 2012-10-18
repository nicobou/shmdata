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

  std::string 
  QuiddityManager::get_properties_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_properties_description,
		       quiddity_name.c_str(),
		       NULL);
    //return life_manager_->get_properties_description (quiddity_name);
  }

  std::string 
  QuiddityManager::get_property_description (std::string quiddity_name, std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property_description,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       NULL);
    //return life_manager_->get_property_description (quiddity_name, property_name);
  }

  bool
  QuiddityManager::set_property (std::string quiddity_name,
				 std::string property_name,
				 std::string property_value)
  {
    std::string res = seq_invoke (QuiddityCommand::set_property,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       property_value.c_str(),
		       NULL);
    if (res == "true")
      return true;
    else 
      return false;
     //return life_manager_->set_property(quiddity_name, property_name,property_value);
  }

  std::string
  QuiddityManager::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       NULL);
    //return life_manager_->get_property(quiddity_name, property_name);
  }

  bool 
  QuiddityManager::invoke (std::string quiddity_name, 
			   std::string function_name,
			   std::vector<std::string> args)
  {
    std::string res;
    g_mutex_lock (seq_mutex_);
    command_.clear();
    command_.set_name (QuiddityCommand::invoke);
    command_.add_arg (quiddity_name);
    command_.add_arg (function_name);
    command_.set_vector_arg (args);
    invoke_in_gmainloop ();
    res = command_.result_[0];
    g_mutex_unlock (seq_mutex_);
    
    if (res == "true")
      return true;
    else 
      return false;
    //return life_manager_->invoke (quiddity_name, function_name, args);
  } 

  std::string
  QuiddityManager::get_methods_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_methods_description, 
		       quiddity_name.c_str(),
		       NULL);
  }
  
  std::string
  QuiddityManager::get_method_description (std::string quiddity_name, std::string method_name)
  {
    return seq_invoke (QuiddityCommand::get_method_description, 
		       quiddity_name.c_str(),
		       method_name.c_str(),
		       NULL);
  }
   
  std::string
  QuiddityManager::create (std::string quiddity_class)
  {
    return seq_invoke (QuiddityCommand::create, 
		       quiddity_class.c_str(),
		       NULL);
  }

  std::string
  QuiddityManager::create (std::string quiddity_class, std::string nick_name)
  {
    return seq_invoke (QuiddityCommand::create_nick_named, 
		       quiddity_class.c_str(), 
		       nick_name.c_str(), 
		       NULL);
  }

  bool
  QuiddityManager::remove (std::string quiddity_name)
  {
    
    std::string res = seq_invoke (QuiddityCommand::remove, 
				  quiddity_name.c_str(),
				  NULL);
    if (res == "true")
      return true;
    else
      return false;
  }

  std::vector<std::string> 
  QuiddityManager::get_classes ()
  {
    //FIXME make this return a json formated std::string
    return life_manager_->get_classes ();
  }
   
  std::vector<std::string> 
  QuiddityManager::get_quiddities ()
  {
    //FIXME make this return a json formated std::string
    return life_manager_->get_instances ();
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
    seq_mutex_ = g_mutex_new ();
  }

  void
  QuiddityManager::clear_command_sync ()
  {
    //FIXME may get the mutex before exiting 
  }

  //works for char * args only. Use NULL sentinel
  std::string
  QuiddityManager::seq_invoke (QuiddityCommand::command command, ...)
  {
    std::string res;
    g_mutex_lock (seq_mutex_);
    va_list vl;
    va_start(vl, command);
    command_.clear();
    command_.set_name (command);
    char *command_arg = va_arg( vl, char *);
    while (command_arg != NULL)
      {
	command_.add_arg (command_arg);
	command_arg = va_arg( vl, char *);
      }
    va_end(vl);
    invoke_in_gmainloop ();
    res = command_.result_[0];
    g_mutex_unlock (seq_mutex_);
    return res;
  }

  void
  QuiddityManager::invoke_in_gmainloop ()
  {
    g_mutex_lock (exec_mutex_);
    g_idle_add ((GSourceFunc) QuiddityManager::gmainloop_run, (gpointer) this);
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
	//TODO
	break;
      case QuiddityCommand::get_quiddities:
	//TODO
	break;
      case QuiddityCommand::create:
	context->command_.result_.push_back (context->life_manager_->create (context->command_.args_[0],
									     context->life_manager_));
	break;
      case QuiddityCommand::create_nick_named:
	context->command_.result_.push_back (context->life_manager_->create (context->command_.args_[0],
									      context->command_.args_[1],
									      context->life_manager_));
	break;
      case QuiddityCommand::remove:
	if (context->life_manager_->remove (context->command_.args_[0]))
	  context->command_.result_.push_back ("true");
	else
	  context->command_.result_.push_back ("false");
	break;
      case QuiddityCommand::get_properties_description:
	context->command_.result_.push_back (context->life_manager_->get_properties_description (context->command_.args_[0]));
	break;
      case QuiddityCommand::get_property_description:
	context->command_.result_.push_back (context->life_manager_->get_property_description (context->command_.args_[0],
											       context->command_.args_[1]));
	break;
      case QuiddityCommand::set_property:
	if (context->life_manager_->set_property (context->command_.args_[0], context->command_.args_[1], context->command_.args_[2]))
	  context->command_.result_.push_back ("true");
	else
	  context->command_.result_.push_back ("false");
	break;
      case QuiddityCommand::get_property:
	context->command_.result_.push_back (context->life_manager_->get_property (context->command_.args_[0],
										   context->command_.args_[1]));
	break;
      case QuiddityCommand::get_methods_description:
	context->command_.result_.push_back (context->life_manager_->get_methods_description (context->command_.args_[0]));
	break;
      case QuiddityCommand::get_method_description:
	context->command_.result_.push_back (context->life_manager_->get_method_description (context->command_.args_[0],
											     context->command_.args_[1]));
	break;
      case QuiddityCommand::invoke:
	if (context->life_manager_->invoke (context->command_.args_[0], context->command_.args_[1], context->command_.vector_arg_))
	  context->command_.result_.push_back ("true");
	else
	  context->command_.result_.push_back ("false");
	break;
      default:
	g_printerr("unknown command");
      }
    g_cond_signal (context->exec_cond_);
    g_mutex_unlock (context->exec_mutex_);
    return FALSE; //this thread should be removed from the main loop
  }
  
  }
