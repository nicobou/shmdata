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

#include "switcher/gobject-custom-property.h"

namespace switcher
{
  GObjectCustomProperty::GObjectCustomProperty ()
  {
  }

  
  GObjectCustomProperty::~GObjectCustomProperty ()
  {
    if (nickname_ != NULL)
      g_free (nickname_);
    if (description_ != NULL)
      g_free (description_);
  }

  GObjectCustomProperty::ptr
  GObjectCustomProperty::make_custom_property (const char *nickname,
					       const char *description,
					       GParamSpec *param_spec,
					       set_method_pointer set_method,
					       get_method_pointer get_method)
  {
    GObjectCustomProperty::ptr custom_prop(new GObjectCustomProperty);
    custom_prop->set_members (nickname,
			      description,
			      param_spec,
			      set_method,
			      get_method);
    return custom_prop;
  }
  
  void
  GObjectCustomProperty::set_members (const char *nickname,
				      const char *description,
				      GParamSpec *param_spec,
				      set_method_pointer set_method,
				      get_method_pointer get_method)
  {
    nickname_ = g_strdup (nickname);
    description_= g_strdup (description);
    param_spec_ = param_spec;
    set_method_ = set_method;
    get_method_ = get_method;
  }

}
