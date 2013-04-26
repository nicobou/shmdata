/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "quiddity-manager.h"
#include "quiddity.h" 

namespace switcher
{

  bool QuiddityManager::gmainloop_initialized_ = false;

  QuiddityManager::ptr 
  QuiddityManager::make_manager ()
  {
    QuiddityManager::ptr manager(new QuiddityManager);
    return manager;
  }
  
  QuiddityManager::ptr 
  QuiddityManager::make_manager (std::string name)
  {
    QuiddityManager::ptr manager(new QuiddityManager(name));
    return manager;
  }

  
  QuiddityManager::QuiddityManager() :
    name_ ("default")
  {
    init_gmainloop ();
    init_command_sync ();
    life_manager_ = QuiddityLifeManager::make_life_manager ();
  }

  QuiddityManager::QuiddityManager(std::string name) :
    name_ (name)
  {
    init_gmainloop ();
    init_command_sync ();
    life_manager_ = QuiddityLifeManager::make_life_manager (name);
  }

  QuiddityManager::~QuiddityManager()
  {
    clear_command_sync ();
    //FIXME count instances and do quit main loop
    //g_main_loop_quit (mainloop_);
    g_debug ("base quiddity manager destructed %s",name_.c_str ());
  }

  std::string
  QuiddityManager::get_name()
  {
    return name_;
  }

  std::string 
  QuiddityManager::get_properties_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_properties_description,
		       quiddity_name.c_str(),
		       NULL);
  }

  std::string 
  QuiddityManager::get_property_description (std::string quiddity_name, std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property_description,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       NULL);
  }

  std::string 
  QuiddityManager::get_properties_description_by_class (std::string class_name)
  {
    return seq_invoke (QuiddityCommand::get_properties_description_by_class,
		       class_name.c_str(),
		       NULL);
  }

  std::string 
  QuiddityManager::get_property_description_by_class (std::string class_name, std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property_description_by_class,
		       class_name.c_str(),
		       property_name.c_str(),
		       NULL);
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
  }

  std::string
  QuiddityManager::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       NULL);
  }


  bool 
  QuiddityManager::make_subscriber (std::string subscriber_name,
				    void (*callback)(std::string subscriber_name,
						     std::string quiddity_name,
						     std::string property_name,
						     std::string value,
						     void *user_data),
				    void *user_data)
  {
    return life_manager_->make_subscriber (subscriber_name, callback, user_data);
  }

    bool 
QuiddityManager::remove_subscriber (std::string subscriber_name)
    {
      return life_manager_->remove_subscriber (subscriber_name);
    }
  
  bool 
  QuiddityManager::subscribe_property (std::string subscriber_name,
				       std::string quiddity_name,
				       std::string property_name)
  {
    return life_manager_->subscribe_property (subscriber_name, quiddity_name, property_name);
  }
  
  bool 
  QuiddityManager::unsubscribe_property (std::string subscriber_name,
					 std::string quiddity_name,
					 std::string property_name)
  {
    return life_manager_->unsubscribe_property (subscriber_name, quiddity_name, property_name);
      }
  
  std::vector<std::string> 
  QuiddityManager::list_subscribers ()
  {
    return life_manager_->list_subscribers ();
  }
  
  std::vector<std::pair<std::string, std::string> > 
  QuiddityManager::list_subscribed_properties (std::string subscriber_name)
  {
    return life_manager_->list_subscribed_properties (subscriber_name);
  }
  
  std::string 
  QuiddityManager::list_subscribers_json ()
  {
    return life_manager_->list_subscribers_json ();
  }

  std::string 
  QuiddityManager::list_subscribed_properties_json (std::string subscriber_name)
  {
    return life_manager_->list_subscribed_properties_json (subscriber_name);
  }
  
  //lower level subscription
  bool
  QuiddityManager::subscribe_property_glib (std::string quiddity_name,
					    std::string property_name,
					    Property::Callback cb, 
					    void *user_data)
  {
    
    return life_manager_->subscribe_property_glib (quiddity_name,
						   property_name,
						   cb,
						   user_data);
  }

  bool
  QuiddityManager::unsubscribe_property_glib (std::string quiddity_name,
					      std::string property_name,
					      Property::Callback cb, 
					      void *user_data)
  {
    return life_manager_->unsubscribe_property_glib (quiddity_name,
						     property_name,
						     cb, 
						     user_data);
  }

  
  bool 
  QuiddityManager::invoke_va (const gchar *quiddity_name, 
			      ...)
  {
    std::vector<std::string> method_args;
    
    if (quiddity_name == NULL)
      {
	g_warning ("trying to invoke with a NULL quiddity name");
	return false;
      }
    va_list vl;
    va_start(vl, quiddity_name);
    char *method_name = va_arg(vl, char *);
    if (method_name == NULL)
      {
	g_warning ("trying to invoke with a NULL method name");
	return false;
      }
    char *method_arg = va_arg(vl, char *);
    while (method_arg != NULL)
      {
	method_args.push_back (method_arg);
	method_arg = va_arg(vl, char *);
      }
    va_end(vl);
    return invoke (quiddity_name, method_name, method_args);
  }

  bool 
  QuiddityManager::invoke (std::string quiddity_name, 
			   std::string method_name,
			   std::vector<std::string> args)
  {
    std::string res;
    g_mutex_lock (seq_mutex_);
    command_.clear();
    command_.set_name (QuiddityCommand::invoke);
    command_.add_arg (quiddity_name);
    command_.add_arg (method_name);
    command_.set_vector_arg (args);
    invoke_in_gmainloop ();
    res = command_.result_[0];
    g_mutex_unlock (seq_mutex_);
    
    if (res == "true")
      return true;
    else 
      return false;
  } 
  
  bool
  QuiddityManager::auto_invoke (std::string method_name,
				std::vector<std::string> args)
  {
    auto_invoke_method_name_ = method_name;
    auto_invoke_args_ = args;
    return true;
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
  QuiddityManager::get_methods_description_by_class (std::string class_name)
  {
    return seq_invoke (QuiddityCommand::get_methods_description_by_class, 
		       class_name.c_str(),
		       NULL);
  }

  std::string
  QuiddityManager::get_method_description_by_class (std::string class_name, 
						    std::string method_name)
  {
    return seq_invoke (QuiddityCommand::get_method_description_by_class, 
		       class_name.c_str(),
		       method_name.c_str(),
		       NULL);
  }

  void 
  QuiddityManager::auto_init (std::string quiddity_name)
  {
    if (!life_manager_->exists (quiddity_name))
      return;
    Quiddity::ptr quidd = life_manager_->get_quiddity (quiddity_name);
    QuiddityManagerWrapper::ptr wrapper = std::dynamic_pointer_cast<QuiddityManagerWrapper> (quidd);
    if (wrapper)
      wrapper->set_quiddity_manager (shared_from_this());
    
    if (!auto_invoke_method_name_.empty ())
      {
	//TODO this should test if the method exists 
	invoke (quidd->get_nick_name (), auto_invoke_method_name_,auto_invoke_args_);
      }
  }
  
  std::string
  QuiddityManager::create (std::string quiddity_class)
  {
    std::string res = seq_invoke (QuiddityCommand::create, 
				  quiddity_class.c_str(),
				  NULL);
    auto_init (res);
    return res;
  }

  std::string
  QuiddityManager::create (std::string quiddity_class, std::string nick_name)
  {
    std::string res= seq_invoke (QuiddityCommand::create_nick_named, 
				 quiddity_class.c_str(), 
				 nick_name.c_str(), 
				 NULL);
    auto_init (res);
    return res;
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
    return life_manager_->get_classes ();
  }

  std::string 
  QuiddityManager::get_classes_doc ()
  {
    return life_manager_->get_classes_doc ();
  }

  
  std::string 
  QuiddityManager::get_class_doc (std::string class_name)
  {
    return life_manager_->get_class_doc (class_name);
  }

  //FIXME make this a command (or not)
  std::string 
  QuiddityManager::get_quiddities_description ()
  {
    return life_manager_->get_quiddities_description ();
  }

  std::string 
  QuiddityManager::get_quiddity_description (std::string quiddity_name)
  {
    return life_manager_->get_quiddity_description (quiddity_name);
  }

 
  std::vector<std::string> 
  QuiddityManager::get_quiddities ()
  {
    return life_manager_->get_instances ();
  }
  
  void
  QuiddityManager::init_gmainloop ()
  {
    if (!gmainloop_initialized_)
      {
	gst_init (NULL,NULL);
	mainloop_ = g_main_loop_new (NULL, FALSE);
	GstRegistry *registry;
	registry = gst_registry_get_default();
	//TODO add option for scanning a path
	gst_registry_scan_path (registry, "/usr/local/lib/gstreamer-0.10/");
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
    g_thread_new (NULL, (GThreadFunc) QuiddityManager::gmainloop_run, (gpointer) this);
    g_cond_wait (exec_cond_, exec_mutex_);
    g_mutex_unlock (exec_mutex_);
  }

  gpointer
  QuiddityManager::gmainloop_run (gpointer user_data)
  {
    QuiddityManager *context = static_cast<QuiddityManager *>(user_data);

    g_mutex_lock (context->exec_mutex_);
    switch (context->command_.name_)
      {
      case QuiddityCommand::get_classes:
	//TODO + other get_class docs
	break;
      case QuiddityCommand::get_quiddities:
	//TODO
	break;
      case QuiddityCommand::create:
	context->command_.result_.push_back (context->life_manager_->create (context->command_.args_[0]));
	break;
      case QuiddityCommand::create_nick_named:
	context->command_.result_.push_back (context->life_manager_->create (context->command_.args_[0], context->command_.args_[1]));
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
	context->command_.result_.push_back (context->life_manager_->get_property_description (context->command_.args_[0], context->command_.args_[1]));
	break;
      case QuiddityCommand::get_properties_description_by_class:
	context->command_.result_.push_back (context->life_manager_->get_properties_description_by_class (context->command_.args_[0]));
	break;
      case QuiddityCommand::get_property_description_by_class:
	context->command_.result_.push_back (context->life_manager_->get_property_description_by_class (context->command_.args_[0], context->command_.args_[1]));
	break;
      case QuiddityCommand::set_property:
	if (context->life_manager_->set_property (context->command_.args_[0], context->command_.args_[1], context->command_.args_[2]))
	  context->command_.result_.push_back ("true");
	else
	  context->command_.result_.push_back ("false");
	break;
      case QuiddityCommand::get_property:
	context->command_.result_.push_back (context->life_manager_->get_property (context->command_.args_[0], context->command_.args_[1]));
	break;
      case QuiddityCommand::get_methods_description:
	context->command_.result_.push_back (context->life_manager_->get_methods_description (context->command_.args_[0]));
	break;
      case QuiddityCommand::get_method_description:
	context->command_.result_.push_back (context->life_manager_->get_method_description (context->command_.args_[0], context->command_.args_[1]));
	break;
      case QuiddityCommand::get_methods_description_by_class:
	context->command_.result_.push_back (context->life_manager_->get_methods_description_by_class (context->command_.args_[0]));
	break;
      case QuiddityCommand::get_method_description_by_class:
	context->command_.result_.push_back (context->life_manager_->get_method_description_by_class (context->command_.args_[0], context->command_.args_[1]));
	break;
      case QuiddityCommand::invoke:
	if (context->life_manager_->invoke (context->command_.args_[0], context->command_.args_[1], context->command_.vector_arg_))
	  context->command_.result_.push_back ("true");
	else
	  context->command_.result_.push_back ("false");
	break;
      default:
	g_error("unknown command");
      }
    g_cond_signal (context->exec_cond_);
    g_mutex_unlock (context->exec_mutex_);
    return NULL;
  }
  
  }
