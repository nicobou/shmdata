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

    static void notify_property_changed (GObject *object, GParamSpec *pspec);
    GObject *get_gobject ();
    //user data for set and get methods
    void set_user_data (std::string nickname, void *user_data);
    void *get_user_data (std::string nickname);
    void set_default_user_data (void *default_user_data);

    //TODO see g_value_... for  implementation of other types
    static 
      GParamSpec *make_int_property (const gchar *nickname, 
				     const gchar *description,
				     gint min_value,
				     gint max_value,
				     gint default_value,
				     GParamFlags read_write_flags,
				     GObjectCustomProperty::set_method_pointer set_method,
				     GObjectCustomProperty::get_method_pointer get_method);
    
    static
      GParamSpec *make_string_property (const gchar *nickname, 
					const gchar *description,
					const gchar *default_value,
					GParamFlags read_write_flags,
					GObjectCustomProperty::set_method_pointer set_method,
					GObjectCustomProperty::get_method_pointer get_method);

    static 
      GParamSpec *make_boolean_property (const gchar *nickname, 
					 const gchar *description,
					 gboolean default_value,
					 GParamFlags read_write_flags,
					 GObjectCustomProperty::set_method_pointer set_method,
					 GObjectCustomProperty::get_method_pointer get_method);


    //for the gobject class
    GObjectCustomProperty::set_method_pointer get_set_method_pointer (guint prop_id);
    GObjectCustomProperty::get_method_pointer get_get_method_pointer (guint prop_id);


  private:
    struct _MyObject *my_object_;
    static std::map<guint, GObjectCustomProperty::ptr> custom_properties_;
    static guint next_prop_id_;
    std::map<std::string, void *> user_datas_;
    void *default_user_data_;
  };

}  // end of namespace

#endif // ifndef
