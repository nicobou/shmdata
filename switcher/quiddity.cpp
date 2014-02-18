/*
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
#include <list>
#include <algorithm> 

namespace switcher
{
  std::map<std::pair <std::string,std::string>, guint> Quiddity::signals_ids_;

  Quiddity::Quiddity () :
    properties_ (),
    disabled_properties_ (),
    properties_description_ (new JSONBuilder()),
    methods_ (),
    disabled_methods_ (),
    methods_description_ (new JSONBuilder()),
    position_weight_counter_ (0),
    signals_ (),
    signals_description_ (new JSONBuilder()),
    name_ (),
    nick_name_ (),
    manager_impl_ (),
    gobject_ (new GObjectWrapper ())
  {
    gobject_->property_set_default_user_data (this);

    GType arg_type[] = {G_TYPE_STRING};
    install_signal_with_class_name ("Quiddity",
				    "On New Property",
				    "on-property-added",
				    "A new property has been installed",
				    Signal::make_arg_description("Quiddity Name",
								 "quiddity_name",
								 "the quiddity name",
								 "Property Name",
								 "property_name",
								 "the property name",
								 NULL),
				    1, 
				    arg_type);

    install_signal_with_class_name ("Quiddity",
				    "On Property Removed",
				    "on-property-removed",
				    "A properties has been uninstalled",
				    Signal::make_arg_description("Quiddity Name",
								 "quiddity_name",
								 "the quiddity name",
								 "Property Name",
								 "property_name",
								 "the property name",
								 NULL),
				    1, 
				    arg_type);

    install_signal_with_class_name ("Quiddity",
				    "On Property reinstalled",
				    "on-property-reinstalled",
				    "A property has been reinstalled",
				    Signal::make_arg_description("Quiddity Name",
								 "quiddity_name",
								 "the quiddity name",
								 "Property Name",
								 "property_name",
								 "the property name",
								 NULL),
				    1, 
				    arg_type);

    install_signal_with_class_name ("Quiddity",
				    "On New Method",
				    "on-method-added",
				    "A new method has been installed",
				    Signal::make_arg_description("Quiddity Name",
								 "quiddity_name",
								 "the quiddity name",
								 "Method Name",
								 "method_name",
								 "the method name",
								 NULL),
				    1, 
				    arg_type);

    install_signal_with_class_name ("Quiddity",
				    "On Method Removed",
				    "on-method-removed",
				    "A method has been uninstalled",
				    Signal::make_arg_description("Quiddity Name",
								 "quiddity_name",
								 "the quiddity name",
								 "Method Name",
								 "method_name",
								 "the method name",
								 NULL),
				    1, 
				    arg_type);

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
	//g_debug ("signals: a signal named %s has already been registered for this class",signal_name.c_str());
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
  Quiddity::register_property (GObject *object, 
			       GParamSpec *pspec, 
			       std::string name_to_give,
			       std::string long_name,
			       std::string signal_to_emit)
  {
    auto it = properties_.find (name_to_give);
    if (properties_.end () != it)
      {
	g_debug ("registering name %s already exists",
		 name_to_give.c_str());
	return false;
      }
    Property::ptr prop (new Property ());
    prop->set_gobject_pspec (object, pspec);
    prop->set_long_name (long_name);
    prop->set_name (name_to_give);
    prop->set_position_weight (position_weight_counter_);
    position_weight_counter_ += 20;
    
    properties_[name_to_give] = prop; 
    signal_emit (signal_to_emit.c_str (), name_to_give.c_str ());
    return true;
  }
  

  bool 
  Quiddity::install_property_by_pspec (GObject *object, 
					GParamSpec *pspec, 
					std::string name_to_give,
					std::string long_name)
  {
    return register_property (object, 
			      pspec,
			      name_to_give, 
			      long_name, 
			      "on-property-added");
  }

  Property::ptr
  Quiddity::get_property_ptr (std::string property_name)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () != it)
      return it->second;
    g_debug ("Quiddity::get_property_ptr %s not found", 
	     property_name.c_str ());
    Property::ptr result;
    return result;
  }

  Method::ptr
  Quiddity::get_method_ptr (std::string method_name)
  {
    auto it = methods_.find (method_name);
    if (methods_.end () != it)
      return it->second;
    Method::ptr result;
    g_debug ("Quiddity::get_method_ptr %s not found", method_name.c_str ());
    return result;
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
  Quiddity::uninstall_property (std::string property_name)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      return false;
    properties_.erase (it);
    signal_emit ("on-property-removed", property_name.c_str ());
    return true; 
  }

  bool 
  Quiddity::enable_property (std::string property_name)
  {
    auto it = disabled_properties_.find (property_name);
    if (disabled_properties_.end () == it)
      return false;
    properties_[property_name] = it->second;
    disabled_properties_.erase (it);
    signal_emit ("on-property-added", property_name.c_str ());
    return true; 
  }

  bool 
  Quiddity::disable_property (std::string property_name)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      return false;
    disabled_properties_[property_name] = it->second;
    properties_.erase (it);
    signal_emit ("on-property-removed", property_name.c_str ());
    return true; 
  }
  
  bool
  Quiddity::install_property (GObject *object, 
			      std::string gobject_property_name, 
			      std::string name_to_give,
			      std::string long_name)
  {
    GParamSpec *pspec = g_object_class_find_property (G_OBJECT_GET_CLASS(object), 
						      gobject_property_name.c_str());
    if (pspec == NULL)
      {
	g_debug ("property not found %s", gobject_property_name.c_str());
	return false;
      }
    
    return install_property_by_pspec (object, pspec, name_to_give, long_name);
  }


  bool
  Quiddity::reinstall_property (GObject *object, 
				std::string gobject_property_name, 
				std::string name,
				std::string long_name)
  {
    auto it = properties_.find (name);
    if (properties_.end () == it)
      return false;
    properties_.erase (it);

    GParamSpec *pspec = 
      g_object_class_find_property (G_OBJECT_GET_CLASS(object), 
				    gobject_property_name.c_str());
    if (pspec == NULL)
      {
	g_debug ("property not found %s", gobject_property_name.c_str());
	return false;
      }
    return register_property (object, pspec, name, long_name, "on-property-reinstalled");
  }

  //return -1 if method not found
  //TODO implement get method and let the manager to call invoke, get_num_args etc...
  int 
  Quiddity::method_get_num_value_args (std::string method_name)
  {
    auto it = methods_.find (method_name);
    if (methods_.end () != it)
      return (int)it->second->get_num_of_value_args(); 
    
    g_debug ("Quiddity::method_get_num_value_args error: method %s not found", 
	     method_name.c_str());
    return -1;
  }

  bool 
  Quiddity::has_method (const std::string method_name)
  {
    return (methods_.end () != methods_.find (method_name)); 
  }
  
  bool
  Quiddity::invoke_method (const std::string method_name,
			   std::string **return_value,
			   const std::vector<std::string> args)
  {
    auto it = methods_.find (method_name);
    if (methods_.end () == it)
      {
	g_debug ("Quiddity::invoke_method error: method %s not found",
		 method_name.c_str());
	return false;
      }

    GValue res = G_VALUE_INIT;
    if (false == it->second->invoke (args, &res))
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
  Quiddity::method_is_registered (std::string method_name)
  {
    return (methods_.end () != methods_.find (method_name)
	    || disabled_methods_.end () != disabled_methods_.find (method_name));
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
    
    if (method_is_registered (method_name))
      {
	g_debug ("registering name %s already exists",
		 method_name.c_str());
	return false;
      }

    Method::ptr meth (new Method ());
    meth->set_method (method, return_type, arg_types, user_data);

    meth->set_position_weight (position_weight_counter_);
    position_weight_counter_ += 20;

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
    auto it = methods_.find (method_name);
    if (methods_.end () == it)
      it = disabled_methods_.find (method_name);

    it->second->set_description (long_name, 
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
    std::vector <Method::ptr> methods;
    for (auto &it: methods_)
      methods.push_back (it.second);
    std::sort (methods.begin (),
	       methods.end (),
	       Categorizable::compare_ptr);
    for (auto &it: methods)
      methods_description_->add_node_value (it->get_json_root_node ());
    methods_description_->end_array ();
    methods_description_->end_object ();
    return methods_description_->get_string (true);
  }

  std::string 
  Quiddity::get_method_description (std::string method_name)
  {
    auto it = methods_.find (method_name);
    if (methods_.end () == it)
      return "{ \"error\" : \" method not found\"}";
    return it->second->get_description ();
  }


  std::string 
  Quiddity::get_properties_description ()
  {
    properties_description_->reset();
    properties_description_->begin_object ();
    properties_description_->set_member_name ("properties");
    properties_description_->begin_array ();
    std::vector <Property::ptr> properties;
    for (auto &it: properties_)
      properties.push_back (it.second);
    std::sort (properties.begin (),
	       properties.end (),
	       Categorizable::compare_ptr);
    for (auto &it: properties)
      properties_description_->add_node_value (it->get_json_root_node ());
    properties_description_->end_array ();
    properties_description_->end_object ();
    
    return properties_description_->get_string (true);
  }

  std::string 
  Quiddity::get_property_description (std::string property_name)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      return "{ \"error\" : \"property not found\" }";
    return it->second->get_description ();
  }

  bool 
  Quiddity::set_property (std::string property_name, std::string value)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      {
	g_debug ("cannot set non existing property (%s)", 
		 property_name.c_str ());
	return false;
      }
    it->second->set (value);
    return true;
  }

  bool 
  Quiddity::has_property (std::string property_name)
  {
    return (properties_.end () != properties_.find (property_name));
  }

  std::string 
  Quiddity::get_property (std::string property_name)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      return "{ \"error\" : \"property not found\" }";
    return it->second->get ();
  }

  bool 
  Quiddity::subscribe_property (std::string property_name, 
				Property::Callback cb,
				void *user_data)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      {
	g_debug ("property not found (%s)", property_name.c_str ());
	return false;
      }
    return it->second->subscribe (cb, user_data);
  }

  bool 
  Quiddity::unsubscribe_property (std::string property_name,
				  Property::Callback cb,
				  void *user_data)
  {
    auto it = properties_.find (property_name);
    if (properties_.end () == it)
      return false;
    it->second->unsubscribe (cb, user_data);
    return true;
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
    
    for(auto &it :signals_) 
      {
	signals_description_->begin_object ();
	signals_description_->add_string_member ("name",it.first.c_str ());
	JsonNode *root_node = it.second->get_json_root_node ();
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
    g_debug ("%s: returning NULL\n", __PRETTY_FUNCTION__);
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
    
    signal_emit ("on-method-added", method_name.c_str ());
    return true;
  }
  
  bool 
  Quiddity::uninstall_method (std::string method_name)
  {
    auto it = methods_.find (method_name);
    if (methods_.end () == it)
      return false;
    methods_.erase (it);
    signal_emit ("on-method-removed", method_name.c_str ());
    return true; 
  }
  
  bool 
  Quiddity::enable_method (std::string method_name)
  {
    auto it = disabled_methods_.find (method_name);
    if (disabled_methods_.end () == it)
      return false;
    methods_[method_name] = it->second;
    disabled_methods_.erase (it);    
    signal_emit ("on-method-added", method_name.c_str ());
    return true; 
  }

  bool 
  Quiddity::disable_method (std::string method_name)
  {
    auto it = methods_.find (method_name);
    if (methods_.end () == it)
      return false;
    disabled_methods_[method_name] = it->second;
    methods_.erase (it);
    signal_emit ("on-method-removed", method_name.c_str ());
    return true; 
  }

}
