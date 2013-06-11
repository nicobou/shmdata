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


#ifndef __SWITCHER_GOBJECT_WRAPPER_H__
#define __SWITCHER_GOBJECT_WRAPPER_H__

#include <memory>
#include <map>
#include <string>
#include <glib-object.h>
#include "gobject-custom-property.h"
//#include "gobject-custom-signal.h"

namespace switcher
{
  struct _MyObject;
  struct _MyObjectClass;

  class GObjectWrapper
  {
  public:
    typedef std::shared_ptr<GObjectWrapper> ptr;
    GObjectWrapper ();
    ~GObjectWrapper ();

    GObject *get_gobject ();

    //---------- properties
    static bool notify_property_changed (GObject *object, GParamSpec *pspec);
    //user data for set and get methods
    void property_set_user_data (std::string nickname, void *user_data);
    void *property_get_user_data (std::string nickname);
    void property_set_default_user_data (void *default_user_data);



    //TODO see g_value_... for  implementation of other types
    static GParamSpec *
      make_int_property (const gchar *nickname, 
			 const gchar *description,
			 gint min_value,
			 gint max_value,
			 gint default_value,
			 GParamFlags read_write_flags,
			 GObjectCustomProperty::set_method_pointer set_method,
			 GObjectCustomProperty::get_method_pointer get_method);
    
    static GParamSpec *
      make_string_property (const gchar *nickname, 
			    const gchar *description,
			    const gchar *default_value,
			    GParamFlags read_write_flags,
			    GObjectCustomProperty::set_method_pointer set_method,
			    GObjectCustomProperty::get_method_pointer get_method);
    
    static GParamSpec *
      make_boolean_property (const gchar *nickname, 
			     const gchar *description,
			     gboolean default_value,
			     GParamFlags read_write_flags,
			     GObjectCustomProperty::set_method_pointer set_method,
			     GObjectCustomProperty::get_method_pointer get_method);
    
    //signal    
    static guint 
      make_signal (GType return_type,
		   guint n_params,
		   GType *param_types);    
    
    static guint 
      make_signal_action (GClosure *class_closure,
			  GType return_type,
			  guint n_params,
			  GType *param_types);
    
    //for the gobject class
    GObjectCustomProperty::set_method_pointer get_set_method_pointer (guint prop_id);
    GObjectCustomProperty::get_method_pointer get_get_method_pointer (guint prop_id);
    
    

  private:
    struct _MyObject *my_object_;

    //---------- properties
    static std::map<guint, GObjectCustomProperty::ptr> custom_properties_;
    static guint next_prop_id_;
    std::map<std::string, void *> property_user_datas_;
    void *property_default_user_data_;
    //---------- signals
    //static std::map<guint, GObjectCustomSignal::ptr> custom_signals_;  
    static guint next_signal_num_; //this is only for generation of unique signal names  
    std::map<std::string, void *> signal_user_datas_;  
    void *signal_default_user_data_;  
  };

}  // end of namespace

#endif // ifndef
