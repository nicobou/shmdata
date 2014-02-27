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


#ifndef __SWITCHER_QUIDDITY_H__
#define __SWITCHER_QUIDDITY_H__

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
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
    
    friend class StartableQuiddity;
    friend class Runtime;

  public:
    typedef std::shared_ptr<Quiddity> ptr;
    Quiddity ();
    Quiddity (const Quiddity &) = delete;
    Quiddity &operator= (const Quiddity &) = delete;
    virtual ~Quiddity ();
    
    //class documentation
    virtual QuiddityDocumentation get_documentation() = 0;
    
    //class initialisation
    virtual bool init () = 0;

    //instance name
    std::string get_name ();
    std::string get_nick_name ();
    bool set_nick_name (std::string nick_name);
    bool set_name (std::string name);

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
    bool has_property (std::string property_name);
    Property::ptr get_property_ptr (std::string property_name);

    //methods
    std::string get_method_description (std::string method_name);
    std::string get_methods_description ();
    bool invoke_method (const std::string function_name,
			std::string **return_value, 
			const std::vector<std::string> args);
    int method_get_num_value_args (std::string function_name); //returns -1 if method not found
    int method_get_num_pointer_args (std::string function_name); //returns -1 if method not found
    bool has_method (const std::string method_name);
    Method::ptr get_method_ptr (std::string method_name);

    //signals
    std::string get_signals_description (); 
    std::string get_signal_description (std::string signal_name); 
    bool subscribe_signal (std::string name,
			   Signal::OnEmittedCallback cb, 
			   void *user_data);
    bool unsubscribe_signal (std::string name,
			     Signal::OnEmittedCallback cb,
			     void *user_data);
    bool emit_action (const std::string signal_name,
			std::string **return_value,
			const std::vector<std::string> args);
   
    //shmdata socket names
    static std::string get_socket_name_prefix ();
    static std::string get_socket_dir ();

    //manager_impl  initialization
    void set_manager_impl (std::shared_ptr<QuiddityManager_Impl> manager_impl);

  private:
    //properties
    std::unordered_map <std::string, Property::ptr> properties_;
    std::unordered_map <std::string, Property::ptr> disabled_properties_;
    JSONBuilder::ptr properties_description_;
    
    //methods
    std::unordered_map <std::string, Method::ptr> methods_;
    std::unordered_map <std::string, Method::ptr> disabled_methods_;
    bool method_is_registered (std::string method_name);
    JSONBuilder::ptr methods_description_;

    //position weight
    gint position_weight_counter_;
    bool compare_properties (std::string first, std::string second);

    //pair is <class_name, signal_name>
    //this map is static in order to avoid re-creation of the same signal for each quiddity instance 
    static std::map<std::pair <std::string,std::string>, guint> signals_ids_;
    std::unordered_map<std::string, Signal::ptr> signals_;
    JSONBuilder::ptr signals_description_;

    //naming
    std::string name_;
    std::string nick_name_;

    bool register_property (GObject *object, 
			    GParamSpec *pspec, 
			    std::string name_to_give,
			    std::string long_name,
			    std::string signal_to_emit);

    //method
    bool register_method (std::string method_name,
			  Method::method_ptr method, 
			  Method::return_type return_type,
			  Method::args_types arg_types, 
			  gpointer user_data);
    
    bool set_method_description (const std::string long_name,
				 const std::string method_name,
				 const std::string short_description,
				 const std::string return_description,
				 const Method::args_doc arg_description);
    
    //category and positions
    bool put_method_in_category (std::string method, std::string category);
    bool set_method_position_weight (std::string method, int position_weight);
    bool put_property_in_category (std::string property, std::string category);
    bool set_property_position_weight (std::string property, int position_weight);

    //signals
    bool register_signal_gobject (const std::string signal_name, //the name to give
				  GObject *object, 
				  const std::string gobject_signal_name);//the internal gobject signal name
    

    //allows for creation of signals in a parent class (like segment)
    bool make_custom_signal_with_class_name (const std::string class_name, //quiddity class name that is making the signal
					     const std::string signal_name, //the name to give
					     GType return_type,
					     guint n_params, //number of params
					     GType *param_types);
    
    bool set_signal_description (const std::string long_name,
				 const std::string signal_name,
				 const std::string short_description,
				 const std::string return_description,
				 const Signal::args_doc arg_description);
    
    //allows for creation of signals in a parent class
    // FIXME the actual method signature in the quiddity should start with a unused void *, such as:
    // static gchar *my_signal_action (void *, gchar *first_arg, void *user_data);
    bool register_signal_action_with_class_name (const std::string class_name,
						 const std::string method_name, //the name to give
						 void *method,
						 GType return_type,
						 guint n_params, //number of params
						 GType *param_types,
						 void *user_data);
    bool register_signal_action (const std::string method_name, //the name to give
				 void *method,
				 GType return_type,
				 guint n_params, //number of params
				 GType *param_types,
				 void *user_data);
   
  protected:
    //property
    bool install_property (GObject *object, 
			    std::string gobject_property_name, 
			    std::string name_to_give,
			    std::string long_name);
    bool reinstall_property (GObject *replacement_object, 
			     std::string gobject_property_name, 
			     std::string name,
			     std::string long_name);
    
    bool install_property_by_pspec (GObject *object, 
				     GParamSpec *pspec, 
				     std::string name_to_give,
				     std::string long_name);
    bool uninstall_property (std::string name);
    //properties are enabled by default during installation
    bool disable_property (std::string name);
    bool enable_property (std::string name);

    //methods
    bool install_method (const std::string long_name,
			 const std::string method_name,
			 const std::string short_description,
			 const std::string return_description,
			 const Method::args_doc arg_description,
			 Method::method_ptr method, 
			 Method::return_type return_type,
			 Method::args_types arg_types, 
			 gpointer user_data);
    
    bool uninstall_method (std::string name);
    bool disable_method (std::string name); 
    bool enable_method (std::string name); 

    //signals 
    bool install_signal (const std::string long_name,
			 const std::string signal_name,
			 const std::string short_description,
			 const Signal::args_doc arg_description,
			 guint number_of_params, 
			 GType *param_types);

    bool install_signal_with_class_name (const std::string class_name,
					 const std::string long_name,
					 const std::string signal_name,
					 const std::string short_description,
					 const Signal::args_doc arg_description,
					 guint number_of_params, 
					 GType *param_types);

    void signal_emit (const std::string signal_name, 
		      ...);

    //custom signals
    void emit_on_interface_changed (); //in order to tell properties/methods has changed
    
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

#define SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(cpp_quiddity_class,	\
					     long_name,			\
					     category,			\
					     short_description,		\
					     license,			\
					     class_name,		\
					     author)			\
  QuiddityDocumentation							\
  cpp_quiddity_class::switcher_doc_ (long_name,				\
				     category,				\
				     short_description,			\
				     license,				\
				     class_name,			\
				     author);				\
  QuiddityDocumentation cpp_quiddity_class::get_documentation ()	\
    {return switcher_doc_;}
  
  
#define SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(cpp_quiddity_class)	\
  typedef std::shared_ptr<cpp_quiddity_class> ptr;			\
  QuiddityDocumentation get_documentation ();				\
  static QuiddityDocumentation switcher_doc_;

#define SWITCHER_DECLARE_PLUGIN(cpp_quiddity_class)             \
  extern "C" Quiddity *create () {		                \
    return new cpp_quiddity_class;				\
  }								\
  extern "C" void destroy(Quiddity *quiddity) {			\
    delete quiddity;						\
  }								\
  extern "C" QuiddityDocumentation get_documentation () {	                \
    return cpp_quiddity_class::switcher_doc_;			\
  }

} // end of namespace

#endif // ifndef
