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

#include "custom-property-helper.h"

namespace switcher
{

  CustomPropertyHelper::CustomPropertyHelper ()
  {
    gobject_.reset (new GObjectWrapper ());
  }

  GObject *
  CustomPropertyHelper::get_gobject ()
  {
    return gobject_->get_gobject ();
  }

  GParamSpec *
  CustomPropertyHelper::make_string_property (const gchar *nickname, 
					      const gchar *description,
					      const gchar *default_value,
					      GParamFlags read_write_flags,
					      set_string_method set_method,
					      get_string_method get_method,
					      void *user_data)
  {

    GParamSpec *pspec = GObjectWrapper::make_string_property (nickname, 
						 description,
						 default_value,
						 read_write_flags,
						 set_by_gvalue,
						 get_by_gvalue);

    make_user_method (nickname,
		      pspec,
		      (void (*) (void))set_method,
		      (void (*) (void))get_method,
		      user_data);
    return pspec; 
  }

  GParamSpec *
  CustomPropertyHelper::make_boolean_property (const gchar *nickname, 
					       const gchar *description,
					       gboolean default_value,
					       GParamFlags read_write_flags,
					       set_boolean_method set_method,
					       get_boolean_method get_method,
					       void *user_data)
  {

    GParamSpec *pspec = GObjectWrapper::make_boolean_property (nickname, 
							       description,
							       default_value,
							       read_write_flags,
							       set_by_gvalue,
							       get_by_gvalue);
    
    make_user_method (nickname,
		      pspec,
		      (void (*) (void))set_method,
		      (void (*) (void))get_method,
		      user_data);
    return pspec; 
  }

  GParamSpec *
  CustomPropertyHelper::make_int_property (const gchar *nickname, 
					   const gchar *description,
					   gint min_value,
					   gint max_value,
					   gint default_value,
					   GParamFlags read_write_flags,
					   set_int_method set_method,
					   get_int_method get_method,
					   void *user_data)
  {
    
    GParamSpec *pspec = GObjectWrapper::make_int_property (nickname, 
							   description,
							   min_value,
							   max_value,
							   default_value,
							   read_write_flags,
							   set_by_gvalue,
							   get_by_gvalue);
    
    make_user_method (nickname,
		      pspec,
		      (void (*) (void))set_method,
		      (void (*) (void))get_method,
		      user_data);
    return pspec; 
  }


  void
  CustomPropertyHelper::make_user_method (const gchar *nickname,
					  GParamSpec *pspec,
					  void (*set_method) (void),
					  void (*get_method) (void),
					  void *user_data)
  {
    std::shared_ptr <UserMethod> user_method (new UserMethod());
    user_method->set = (void (*) (void))set_method;
    user_method->get = (void (*) (void))get_method;
    user_method->user_data = user_data;
    user_method->gobject = get_gobject ();
    user_method->pspec = pspec;
    user_methods_.push_back (user_method);
    gobject_->set_user_data (nickname, user_method.get ());
  }

  bool
  CustomPropertyHelper::get_by_gvalue (GValue *value,
				       void *user_data)
  {
    UserMethod *user_method = static_cast<UserMethod *> (user_data);
    
    if (G_VALUE_TYPE(value) == G_TYPE_STRING)
      {
	gchar *val = ((get_string_method)user_method->get) (user_method->user_data);
	g_value_set_string (value, val);
      }
    else if (G_VALUE_TYPE(value) == G_TYPE_BOOLEAN)
      {
	gboolean val = ((get_boolean_method)user_method->get) (user_method->user_data);
	g_value_set_boolean (value, val);
      }
    else
      g_warning ("CustomPropertyHelper: unknown type"); 
    return TRUE;
  }
 
  bool
  CustomPropertyHelper::set_by_gvalue (const GValue *value, void *user_data)
  {
    UserMethod *user_method = static_cast<UserMethod *> (user_data);
    
    if (G_VALUE_TYPE(value) == G_TYPE_STRING)
      {
	((set_string_method)user_method->set) (g_value_get_string (value), 
					       user_method->user_data);
      }
    else if (G_VALUE_TYPE(value) == G_TYPE_BOOLEAN)
      {
	((set_boolean_method)user_method->set) (g_value_get_boolean (value), 
						user_method->user_data);
      }
    else
      {
	g_warning ("CustomPropertyHelper: unknown type"); 
	return FALSE;
      }
    GObjectWrapper::notify_property_changed (user_method->gobject,
					     user_method->pspec);
    return TRUE;
  }

  void
  CustomPropertyHelper::notify_property_changed (GParamSpec *pspec)
  {
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (),
					     pspec);
  }

}
