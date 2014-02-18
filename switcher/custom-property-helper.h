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

#ifndef __SWITCHER_CUSTOM_PROPERTY_HELPER_H__
#define __SWITCHER_CUSTOM_PROPERTY_HELPER_H__

#include <memory>
#include <vector>
#include "gobject-wrapper.h"

namespace switcher
{
  //you may get better performances are achieved using gobject wrapper with static paramspec
  //since using the helper installs a new paramspec for each instance

  class CustomPropertyHelper
  {
  public:
    typedef std::shared_ptr<CustomPropertyHelper> ptr;
    typedef void (*set_string_method)(const gchar *value, void *user_data);
    typedef const gchar *(*get_string_method)(void *user_data);
    typedef void (*set_boolean_method)(const gboolean value, void *user_data);
    typedef gboolean (*get_boolean_method)(void *user_data);
    typedef void (*set_int_method)(const gint value, void *user_data);
    typedef gint (*get_int_method)(void *user_data);
    typedef void (*set_double_method)(const gdouble value, void *user_data);
    typedef gdouble (*get_double_method)(void *user_data);
    typedef void (*set_enum_method)(const gint value, void *user_data);
    typedef gint(*get_enum_method)(void *user_data);


    typedef struct {
      void (*set) (void);
      void (*get) (void);
      void *user_data;
      GObject *gobject;
      GParamSpec *pspec;
    } UserMethod;

    CustomPropertyHelper ();
    CustomPropertyHelper (const CustomPropertyHelper &) = delete;
    CustomPropertyHelper &operator= (const CustomPropertyHelper &) = delete;

    bool is_property_nickname_taken (std::string nickname);
    bool notify_property_changed (GParamSpec *pspec);
    GObject *get_gobject ();
    GParamSpec *
      make_string_property (const gchar *nickname, 
			    const gchar *description,
			    const gchar *default_value,
			    GParamFlags read_write_flags,
			    set_string_method set_method,
			    get_string_method get_method,
			    void *user_data);
      
    GParamSpec *
      make_boolean_property (const gchar *nickname, 
			     const gchar *description,
			     gboolean default_value,
			     GParamFlags read_write_flags,
			     set_boolean_method set_method,
			     get_boolean_method get_method,
			     void *user_data);

    GParamSpec *
      make_int_property (const gchar *nickname, 
			 const gchar *description,
			 gint min_value,
			 gint max_value,
			 gint default_value,
			 GParamFlags read_write_flags,
			 set_int_method set_method,
			 get_int_method get_method,
			 void *user_data);

    GParamSpec *
      make_double_property (const gchar *nickname, 
			    const gchar *description,
			    gdouble min_value,
			    gdouble max_value,
			    gdouble default_value,
			    GParamFlags read_write_flags,
			    set_double_method set_method,
			    get_double_method get_method,
			    void *user_data);
    
    GParamSpec *
      make_enum_property (const gchar *nickname, 
			  const gchar *description,
			  const gint default_value, //map key
			  const GEnumValue *string_map_enum,
			  GParamFlags read_write_flags,
			  set_enum_method set_method,
			  get_enum_method get_method,
			  void *user_data);
    
    static bool get_by_gvalue (GValue *value, void *user_data);
    static bool set_by_gvalue (const GValue *val, void *user_data);
    
  private:
    GObjectWrapper::ptr gobject_;
    std::vector< std::shared_ptr<UserMethod> > user_methods_;
    void make_user_method (const gchar *nickname,
			   GParamSpec *pspec,
			   void (*set_method) (void),
			   void (*get_method) (void),
			   void *user_data);
  };
  
}  // end of namespace

#endif // ifndef
