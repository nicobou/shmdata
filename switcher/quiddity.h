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

/**
 * The Quiddity class
 */


#ifndef __SWITCHER_QUIDDITY_H__
#define __SWITCHER_QUIDDITY_H__

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <gst/gst.h>

#include "property.h"
#include "method.h"
#include "signal-string.h"
#include "quiddity-documentation.h"
#include "quiddity-manager-impl.h"
#include "json-builder.h"
#include "gobject-wrapper.h"

namespace switcher
{
  class QuiddityManager_Impl;
 
  class Quiddity
  {
  public:
    typedef std::shared_ptr<Quiddity> ptr;
    
    Quiddity ();
    virtual ~Quiddity ();
    
    //class documentation
    virtual QuiddityDocumentation get_documentation() = 0;
    
    //class initialisation
    virtual bool init () = 0;

    //instance name
    std::string get_name ();
    std::string get_nick_name ();
    bool set_nick_name (std::string nick_name);

    //properties
    std::string get_property_description (std::string property_name);
    std::string get_properties_description ();
    bool set_property (std::string name, 
		       std::string value);
    std::string get_property (std::string name);
    bool subscribe_property (std::string name,
			     Property::Callback cb, 
			     void *user_data);
    bool unsubscribe_property (std::string name,
			       Property::Callback cb,
			       void *user_data);
    
    //methods
    std::string get_method_description (std::string method_name);
    std::string get_methods_description ();
    bool invoke_method (std::string function_name,
			std::vector<std::string> args);
    int method_get_num_value_args (std::string function_name); //returns -1 if method not found
    int method_get_num_pointer_args (std::string function_name); //returns -1 if method not found
    bool has_method (const std::string method_name);

    //signals
    std::string get_signals_description (); 
    std::string get_signal_description (std::string signal_name); 
    bool subscribe_signal (std::string name,
			   Signal::OnEmittedCallback cb, 
			   void *user_data);
    bool unsubscribe_signal (std::string name,
			     Signal::OnEmittedCallback cb,
			     void *user_data);
    

    //shmdata socket names
    static std::string get_socket_name_prefix ();
    static std::string get_socket_dir ();

    //manager_impl  initialization
    void set_manager_impl (std::shared_ptr<QuiddityManager_Impl> manager_impl);

  private:
    //properties are registered by derived class
    std::map<std::string, Property::ptr> properties_;
    JSONBuilder::ptr properties_description_;
    std::map<std::string, Method::ptr> methods_;
    JSONBuilder::ptr methods_description_;
    //pair is <class_name, signal_name>
    //this map is static in order to avoid re-creation of the same signal for each quiddity instance 
    static std::map<std::pair <std::string,std::string>, guint> signals_ids_;
    std::map<std::string, Signal::ptr> signals_;
    JSONBuilder::ptr signals_description_;
    std::string name_;
    std::string nick_name_;
 
  protected:
    //naming
    bool set_name (std::string name);

    //property
    bool register_property (GObject *object, 
			    std::string gobject_property_name, 
			    std::string name_to_give);
    bool register_property_by_pspec (GObject *object, 
				     GParamSpec *pspec, 
				     std::string name_to_give);
    //method
    bool register_method (std::string method_name,
			  void *method, 
			  Method::args_types arg_types, 
			  gpointer user_data);

    bool set_method_description (const std::string method_name,
				 const std::string short_description,
				 const Method::args_doc arg_description);

    //signal
    bool register_signal_gobject (const std::string signal_name, //the name to give
				  GObject *object, 
				  const std::string gobject_signal_name);//the internal gobject signal name

    bool make_custom_signal (const std::string signal_name, //the name to give
			     GType return_type,
			     guint n_params, //number of params
			     GType *param_types);

    bool set_signal_description (const std::string signal_name,
				 const std::string short_description,
				 const Signal::args_doc arg_description);

    //following method allows for creation of signals in abstract class like segment
    bool make_custom_signal (const std::string class_name, //quiddity class name that is making the signal
			     const std::string signal_name, //the name to give
			     GType return_type,
			     guint n_params, //number of params
			     GType *param_types);

    void signal_emit (const std::string signal_name, 
		      ...);
    
    //use a consistent naming for shmdatas FIXME move that to segment (or not?) 
    std::string make_file_name (std::string suffix);

    //used in order to dynamically create other quiddity, weak_ptr is used in order to 
    //avoid circular references to the manager_impl 
    std::weak_ptr<QuiddityManager_Impl> manager_impl_;

    //gobject wrapper for custom signals and properties
    GObjectWrapper::ptr gobject_;
    
    //g_main_context
    GMainContext *get_g_main_context ();
  };
  
} // end of namespace

#endif // ifndef
