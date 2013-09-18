/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * The Quiddity class
 */

#include "quiddity.h"
#include "quiddity-manager-impl.h"
#include "gst-utils.h"

namespace switcher
{
  std::map<std::pair <std::string,std::string>, guint> Quiddity::signals_ids_;

  Quiddity::Quiddity ()
  {
    gobject_.reset (new GObjectWrapper ());
    gobject_->property_set_default_user_data (this);
    properties_description_.reset (new JSONBuilder());
    methods_description_.reset (new JSONBuilder());
    signals_description_.reset (new JSONBuilder());
    
    // GType types[] = {G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING};
    // make_custom_signal ("quiddity",
    // 			 "on-new-signal-registered", 
    // 			 G_TYPE_NONE,
    // 			 3,
    // 			 types);
    // set_signal_description ("on-new-signal-registered",
    // 			     "a new signal has been registered and documented",
    // 			     Signal::make_arg_description("quiddity_name",
    // 							  "the quiddity name",
    // 							  "signal_name",
    // 							  "the signal name",
    // 							  "json_doc",
    // 							  "the json-formated signal documentation",
    // 							  NULL));
  }
  
  Quiddity::~Quiddity () 
  {}

  std::string
  Quiddity::get_name()
  {
    return std::string (name_);
  }

  std::string
  Quiddity::get_nick_name()
  {
    return std::string (nick_name_);
  }

  bool
  Quiddity::set_name(std::string name)
  {
    name_ = name;
    if (nick_name_.empty ()) 
      nick_name_ = name;
    return true;
  }

  bool
  Quiddity::set_nick_name(std::string nick_name)
  {
    nick_name_ = nick_name;
    return true;
  }
  
  
  bool 
  Quiddity::register_signal_gobject (std::string signal_name,
				     GObject *object, 
				     std::string gobject_signal_name)
  {
    if (signals_.find(signal_name) != signals_.end())
      {
	g_warning ("signals: a signal named %s has already been registered for this class",signal_name.c_str());
	return false;
      }
    
    Signal::ptr signal;
    signal.reset (new Signal ());
    if (!signal->set_gobject_signame (object, gobject_signal_name))
      return false;
    signals_[signal_name] = signal; 
    g_debug ("signal %s registered with name %s", 
      	     gobject_signal_name.c_str (),
      	     signal_name.c_str ());
    return true;
  }


  bool 
  Quiddity::install_signal (const std::string long_name,
			    const std::string signal_name,
			    const std::string short_description,
			    const Signal::args_doc arg_description,
			    guint number_of_params, 
			    GType *param_types)
  {
    if (!make_custom_signal_with_class_name (get_documentation().get_class_name (),
					     signal_name,
					     G_TYPE_NONE,
					     number_of_params,
					     param_types))
      return false;
    
    if (!set_signal_description (long_name,
				 signal_name,
				 short_description,
				 "n/a",
				 arg_description))
      return false;
    
    return true;  
  }

  bool 
  Quiddity::install_signal_with_class_name (const std::string class_name,
					    const std::string long_name,
					    const std::string signal_name,
					    const std::string short_description,
					    const Signal::args_doc arg_description,
					    guint number_of_params, 
					    GType *param_types)
  {
    if (!make_custom_signal_with_class_name (class_name,
					     signal_name,
					     G_TYPE_NONE,
					     number_of_params,
					     param_types))
      return false;
    
    if (!set_signal_description (long_name,
				 signal_name,
				 short_description,
				 "n/a",
				 arg_description))
      return false;
    
    return true;  
  }

  bool 
  Quiddity::make_custom_signal_with_class_name (const std::string class_name,
						const std::string signal_name, //the name to give
						GType return_type,
						guint n_params, //number of params
						GType *param_types)
  {
    if (signals_.find(signal_name) != signals_.end())
      {
	g_warning ("signals: a signal named %s has already been registered for this class",signal_name.c_str());
	return false;
      }
    
    std::pair <std::string,std::string> sig_pair = std::make_pair (class_name,
								   signal_name);
    if (signals_ids_.find(sig_pair) == signals_ids_.end())
      {
	guint id = GObjectWrapper::make_signal (return_type,
						n_params,
						param_types); 
	if (id == 0)
	  {
	    g_warning ("custom signal %s not created because of a type issue",
		       signal_name.c_str ());
	    return false;
	  }
	signals_ids_[sig_pair] = id;
      }

    Signal::ptr signal (new Signal ());
    if (!signal->set_gobject_sigid (gobject_->get_gobject (), signals_ids_[sig_pair]))
      return false;
    signals_[signal_name] = signal; 
    g_debug ("signal %s registered", 
     	     signal_name.c_str ());
    return true;
  }

  bool 
  Quiddity::set_signal_description (const std::string long_name,
				    const std::string signal_name,
				    const std::string short_description,
				    const std::string return_description,
				    const Signal::args_doc arg_description)
  {

    if (signals_.find(signal_name) == signals_.end())
      {
	g_warning ("cannot set description of a not existing signal");
	return false;
      }
    signals_[signal_name]->set_description (long_name, 
					    signal_name, 
					    short_description, 
					    return_description, 
					    arg_description);

    // signal_emit ("on-new-signal-registered", 
    // 		 get_nick_name ().c_str (), 
    // 		 signal_name.c_str (),
    // 		 (JSONBuilder::get_string (signals_[signal_name]->get_json_root_node (), true)).c_str ());
    return true;
  }


  bool 
  Quiddity::install_property_by_pspec (GObject *object, 
					GParamSpec *pspec, 
					std::string name_to_give,
					std::string long_name)
  {
    if (properties_.contains (name_to_give)) 
      {
	g_debug ("registering name %s already exists",
		 name_to_give.c_str());
	return false;
      }
    
    Property::ptr prop (new Property ());
    prop->set_gobject_pspec (object, pspec, long_name);
    
    properties_.insert (name_to_give, prop); 
    return true;
  }

  Property::ptr
  Quiddity::get_property_ptr (std::string property_name)
  {
    Property::ptr res;
    if (!properties_.contains (property_name))
      g_debug ("Quiddity::get_property_ptr %s not found", property_name.c_str ());
      
    res = properties_.lookup_full (property_name);
    return res;
  }

  bool 
  Quiddity::register_signal_action_with_class_name (const std::string class_name,
						    const std::string method_name, //the name to give
						    void *method,
						    GType return_type,
						    guint n_params, //number of params
						    GType *param_types,
						    void *user_data)
  {
    if (signals_.find(method_name) != signals_.end())
      {
	g_warning ("a custom method named %s has already been registered for this class",
		   method_name.c_str());
	return false;
      }
    if (method == NULL)
      {
	g_warning ("cannot register a NULL method (for %s)", method_name.c_str());
	return false;
      }
      
    std::pair <std::string,std::string> sig_pair = std::make_pair (class_name,
								   method_name);
    GClosure *closure;

    //using signal ids in order to avoid id conflicts between signal and methods
    if (signals_ids_.find(sig_pair) == signals_ids_.end())
      {
	closure = g_cclosure_new (G_CALLBACK (method), user_data, NULL/*destroy data*/);
	g_closure_set_marshal  (closure, g_cclosure_marshal_generic);

	guint id = GObjectWrapper::make_signal_action (closure,
						       return_type,
						       n_params,
						       param_types); 
	if (id == 0)
	  {
	    g_warning ("custom signal %s not created because of a type issue",
		       method_name.c_str ());
	    return false;
	  }
	signals_ids_[sig_pair] = id;
      }

    Signal::ptr signal (new Signal ());
    if (!signal->set_gobject_sigid (gobject_->get_gobject (), signals_ids_[sig_pair]))
      return false;
    signals_[method_name] = signal; 
    g_debug ("signal %s registered", 
     	     method_name.c_str ());
    return true;
  }

  bool 
  Quiddity::register_signal_action (const std::string method_name, //the name to give
				    void *method,
				    GType return_type,
				    guint n_params, //number of params
				    GType *param_types,
				    void *user_data)
  {

    return register_signal_action_with_class_name (get_documentation().get_class_name (),
						   method_name,
						   method,
						   return_type,
						   n_params,
						   param_types,
						   user_data);
  }

 
  bool 
  Quiddity::uninstall_property (std::string name)
  {
    return properties_.remove (name); 
  }
  
  bool
  Quiddity::install_property (GObject *object, 
			       std::string gobject_property_name, 
			       std::string name_to_give,
			       std::string long_name)
  {
    GParamSpec *pspec = g_object_class_find_property (G_OBJECT_GET_CLASS(object), gobject_property_name.c_str());
    if (pspec == NULL)
      {
	g_debug ("property not found %s", gobject_property_name.c_str());
	return false;
      }
    
    return install_property_by_pspec (object, pspec, name_to_give, long_name);
  }

  

  //return -1 if method not found
  //TODO implement get method and let the manager to call invoke, get_num_args etc...
  int 
  Quiddity::method_get_num_value_args (std::string method_name)
  {
    if (methods_.find(method_name ) == methods_.end())
      {
	g_debug ("Quiddity::method_get_num_value_args error: method %s not found", method_name.c_str());
	return -1;
      }
    else 
      return (int)methods_[method_name]->get_num_of_value_args(); 
  }

  bool 
  Quiddity::has_method (const std::string method_name)
  {
    if (methods_.find(method_name) == methods_.end())
      return false;
    return true;
  }
  
  bool
  Quiddity::invoke_method (const std::string method_name,
			   std::string **return_value,
			   const std::vector<std::string> args)
  {
    if (methods_.find(method_name) == methods_.end())
      {
	g_debug ("Quiddity::invoke_method error: method %s not found",
		 method_name.c_str());
	return false;
      }
    else 
      {
	GValue res = G_VALUE_INIT;
	if (!methods_[method_name]->invoke (args, &res))
	  {
	    g_debug ("invokation of %s failled (missing argments ?)",
		     method_name.c_str ());
	    return false;
	  }

	if (return_value != NULL)
	  {
	    gchar *res_val = GstUtils::gvalue_serialize (&res);
	    *return_value = new std::string (res_val);
	    g_free (res_val);
	  }
	g_value_unset (&res);
	return true;
      }
  }
  
  bool 
  Quiddity::emit_action (const std::string signal_name,
			 std::string **return_value,
			 const std::vector<std::string> args)
  {
    if (signals_.find(signal_name) == signals_.end())
      {
	g_debug ("Quiddity::invoke_signal error: %s not found",
		 signal_name.c_str());
	return false;
      }
    else 
      {
	GValue res = signals_[signal_name]->action_emit (args);
	if (return_value != NULL)
	  {
	    gchar *res_val = GstUtils::gvalue_serialize (&res);
	    //gchar *res_val;
	    // if (G_VALUE_HOLDS_STRING(&res))
	    //   res_val = g_strdup (g_value_get_string (&res));
	    // else
	    //   res_val = gst_value_serialize (&res);

	    *return_value = new std::string (res_val);
	    g_free (res_val);
	  }
	g_value_unset (&res);
	return true;
      }
  }
  

  bool
  Quiddity::register_method (std::string method_name, 
			     Method::method_ptr method, 
			     Method::return_type return_type,
			     Method::args_types arg_types, 
			     gpointer user_data)
  {
    if (method == NULL)
      {
	g_debug ("fail registering %s (method is NULL)",method_name.c_str());
	return false;
      }
    
    Method::ptr meth (new Method ());
    meth->set_method (method, return_type, arg_types, user_data);

    if (methods_.find( method_name ) != methods_.end())
      {
	g_debug ("registering name %s already exists",method_name.c_str());
	return false;
      }
    methods_[method_name] = meth;
    return true;
  }

  bool 
  Quiddity::set_method_description (const std::string long_name,
				    const std::string method_name,
				    const std::string short_description,
				    const std::string return_description,
				    const Method::args_doc arg_description)
  {
    if (methods_.find( method_name ) == methods_.end())
      {
	g_debug ("cannot set description of a not existing method");
	return false;
      }
    methods_[method_name]->set_description (long_name, 
					    method_name, 
					    short_description, 
					    return_description,
					    arg_description);
    return true;
  }

  std::string 
  Quiddity::get_methods_description ()
  {
    
    methods_description_->reset();
    methods_description_->begin_object ();
    methods_description_->set_member_name ("methods");
    methods_description_->begin_array ();
    for(std::map<std::string, Method::ptr>::iterator it = methods_.begin(); it != methods_.end(); ++it) 
      methods_description_->add_node_value (it->second->get_json_root_node ());
    methods_description_->end_array ();
    methods_description_->end_object ();
    return methods_description_->get_string (true);
  }

  std::string 
  Quiddity::get_method_description (std::string method_name)
  {
    if (methods_.find( method_name ) == methods_.end())
      return "";
    
    Method::ptr meth = methods_[method_name];
    return meth->get_description ();
  }

  std::string 
  Quiddity::get_properties_description ()
  {
    properties_description_->reset();
    properties_description_->begin_object ();
    properties_description_->set_member_name ("properties");
    properties_description_->begin_array ();

    std::map <std::string, Property::ptr> props = properties_.get_map ();
    for(auto &it: props) 
      {
     	properties_description_->begin_object ();
     	properties_description_->add_string_member ("name",it.first.c_str ());
     	JsonNode *root_node = it.second->get_json_root_node ();
     	properties_description_->add_JsonNode_member ("description", root_node);
     	properties_description_->end_object ();
      }
    
    properties_description_->end_array ();
    properties_description_->end_object ();
    
    return properties_description_->get_string (true);
  }

  std::string 
  Quiddity::get_property_description (std::string name)
  {
    if (!properties_.is_enabled (name))
      return "{ \"error\" : \"property not found\" }";
    
    Property::ptr prop = properties_.lookup (name);
    return prop->get_description ();
  }

  bool 
  Quiddity::set_property (std::string name, std::string value)
  {
    if (!properties_.is_enabled (name))
      return false;

    Property::ptr prop = properties_.lookup (name);
    prop->set (value);
    return true;
  }

  bool 
  Quiddity::has_property (std::string name)
  {
    if (!properties_.is_enabled (name))
      return false;
    return true;
  }

  std::string 
  Quiddity::get_property (std::string name)
  {
    if (!properties_.is_enabled (name))
      return "{ \"error\" : \"property not found\" }";

    Property::ptr prop = properties_.lookup (name);
    return prop->get ();
  }

  bool 
  Quiddity::subscribe_property (std::string name, 
				Property::Callback cb,
				void *user_data)
  {
    if (!properties_.contains (name))
      {
	g_debug ("property not found (%s)", name.c_str ());
	return false;
      }

    Property::ptr prop = properties_.lookup_full (name);
    return prop->subscribe (cb, user_data);
  }

  bool 
  Quiddity::unsubscribe_property (std::string name,
				  Property::Callback cb,
				  void *user_data)
  {
    if (!properties_.contains (name))
      return false;
    
    Property::ptr prop = properties_.lookup_full (name);
    return prop->unsubscribe (cb, user_data);
  }

  bool 
  Quiddity::subscribe_signal (std::string signal_name, 
			      Signal::OnEmittedCallback cb,
			      void *user_data)
  {

    if (signals_.find(signal_name) == signals_.end())
      {
	g_warning ("Quiddity::subscribe_signal, signal %s not found", signal_name.c_str ());
	return false;
      }
    Signal::ptr sig = signals_[signal_name];
    return sig->subscribe (cb, user_data);
  }

  bool 
  Quiddity::unsubscribe_signal (std::string signal_name,
				Signal::OnEmittedCallback cb,
				void *user_data)
  {
    if (signals_.find (signal_name) == signals_.end())
      return false;

    Signal::ptr signal = signals_[signal_name];
    return signal->unsubscribe (cb, user_data);
  }

  void
  Quiddity::signal_emit (const std::string signal_name, 
			 ...)
  {
    if (signals_.find (signal_name) == signals_.end())
      return;
    Signal::ptr signal = signals_[signal_name];
    va_list var_args;
    va_start (var_args, signal_name);
    // va_list va_cp;
    // va_copy (va_cp, var_args);
    // signal->signal_emit (get_g_main_context (), signal_name.c_str (), va_cp); 
    signal->signal_emit (/*get_g_main_context (), */signal_name.c_str (), var_args); 
    va_end (var_args);
  }

  std::string 
  Quiddity::get_signals_description ()
  {

    std::string class_name = get_documentation().get_class_name ();

    signals_description_->reset();
    signals_description_->begin_object ();
    signals_description_->set_member_name ("signals");
    signals_description_->begin_array ();
    
    for(std::map<std::string, Signal::ptr>::iterator it = signals_.begin(); 
     	it != signals_.end(); 
     	++it) 
      {
	signals_description_->begin_object ();
	signals_description_->add_string_member ("name",it->first.c_str ());
	JsonNode *root_node = it->second->get_json_root_node ();
	if (root_node != NULL)
	  signals_description_->add_JsonNode_member ("description", root_node);
	else
	  signals_description_->add_string_member ("description","missing description");
	signals_description_->end_object ();
      }
    
    signals_description_->end_array ();
    signals_description_->end_object ();
    
    return signals_description_->get_string (true);
  }

  std::string 
  Quiddity::get_signal_description (std::string signal_name)
  {
    
    if (signals_.find(signal_name) == signals_.end())
      return "";
    
    Signal::ptr sig = signals_[signal_name];
    return sig->get_description ();
  }


  std::string
  Quiddity::make_file_name (std::string suffix)
  {
    std::string connector_name;
    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if ( (bool)manager)
      connector_name.append ("/tmp/switcher_"+manager->get_name ()+"_"+nick_name_+"_"+suffix);
    else
      connector_name.append ("/tmp/switcher__"+nick_name_+"_"+ suffix); 

    return connector_name;
  }

  std::string
  Quiddity::get_socket_name_prefix ()
  {
    return "switcher_";
  }

  std::string
  Quiddity::get_socket_dir ()
  {
    return "/tmp";
  }


  void
  Quiddity::set_manager_impl (QuiddityManager_Impl::ptr manager_impl)
  {
    manager_impl_ = manager_impl;
  }

  GMainContext *
  Quiddity::get_g_main_context ()
  {
    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if ((bool) manager)
      return manager->get_g_main_context ();
    return NULL;
  }

  //methods
  bool 
  Quiddity::install_method (const std::string long_name,
			    const std::string method_name,
			    const std::string short_description,
			    const std::string return_description,
			    const Method::args_doc arg_description,
			    Method::method_ptr method, 
			    Method::return_type return_type,
			    Method::args_types arg_types, 
			    gpointer user_data)
  {
    if (!register_method (method_name,
			  method, 
			  return_type,
			  arg_types, 
			  user_data))
      return false;

    if (!set_method_description (long_name,
				 method_name,
				 short_description,
				 return_description,
				 arg_description))
      return false;
    return true;
  }
  
}
