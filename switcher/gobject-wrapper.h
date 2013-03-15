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
#include <vector>
#include <string>
#include <glib-object.h>
#include "switcher/gobject-custom-property.h"

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

    bool install_int_property (std::string nickname, 
			       std::string description);
  

  private:
    struct _MyObject *my_object_;
    std::vector<GObjectCustomProperty::ptr> custom_properties_;
    static int next_prop_id_;

    //    static GType my_object_get_type (void); 
    /* static void my_object_set_foo (struct _MyObject *obj, gint foo);  */
    /* static void my_object_set_bar (struct _MyObject *obj, gboolean bar);  */
    /* static void my_object_set_baz (struct _MyObject *obj, const gchar *baz);  */
    /* static void my_object_finalize (GObject *gobject);  */
    /* static void my_object_set_property (GObject *gobject,  */
    /*  					guint prop_id,  */
    /*  					const GValue *value,  */
    /*  					GParamSpec *pspec);  */
    /* static void my_object_get_property (GObject *gobject,  */
    /*  					guint prop_id,  */
    /*  					GValue *value,  */
    /*  					GParamSpec *pspec);  */
    //static void my_object_class_init (struct _MyObjectClass *klass); 
    //static void my_object_init (struct _MyObject *self);
  };

}  // end of namespace

#endif // ifndef
