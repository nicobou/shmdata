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


#ifndef __SWITCHER_GOBJECT_CUSTOM_PROPERTY_H__
#define __SWITCHER_GOBJECT_CUSTOM_PROPERTY_H__

#include <memory>
#include <string>
#include <glib-object.h>

namespace switcher
{
  class GObjectCustomProperty
  {
  public:
    typedef std::shared_ptr<GObjectCustomProperty> ptr;
    typedef bool (*set_method_pointer) (const GValue *val, void *user_data);
    typedef bool (*get_method_pointer) (GValue *val, void *user_data);
    ~GObjectCustomProperty ();
    
    static GObjectCustomProperty::ptr 
      make_custom_property (set_method_pointer set_method,
			    get_method_pointer get_method);

    set_method_pointer set_method_;
    get_method_pointer get_method_;
    
  private:
    GObjectCustomProperty ();
    void set_members (set_method_pointer set_method,
		      get_method_pointer get_method);
  };
}  // end of namespace

#endif // ifndef
