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

#include "gobject-custom-property.h"

namespace switcher
{
  GObjectCustomProperty::GObjectCustomProperty ()
  {
  }

  
  GObjectCustomProperty::~GObjectCustomProperty ()
  {
  }

  GObjectCustomProperty::ptr
  GObjectCustomProperty::make_custom_property (set_method_pointer set_method,
					       get_method_pointer get_method)
  {
    GObjectCustomProperty::ptr custom_prop(new GObjectCustomProperty);
    custom_prop->set_members (set_method,
			      get_method);
    return custom_prop;
  }
  
  void
  GObjectCustomProperty::set_members (set_method_pointer set_method,
				      get_method_pointer get_method)
  {
    set_method_ = set_method;
    get_method_ = get_method;
  }

}
