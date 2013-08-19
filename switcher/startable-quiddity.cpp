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

#include "startable-quiddity.h"

namespace switcher
{

  StartableQuiddity::StartableQuiddity ()
  {
    publish_method ("Start",
		    "start", 
		    "start processing", 
		    "success or fail",
		    Method::make_arg_description ("none",
						  NULL),
		    (Method::method_ptr) &start_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    this);

    publish_method ("Stop",
		    "stop", 
		    "stop processing", 
		    "success or fail",
		    Method::make_arg_description ("none",
						  NULL),
		    (Method::method_ptr) &stop_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    this);

  }

  bool 
  StartableQuiddity::start_wrapped (gpointer user_data)
  {
    StartableQuiddity *context= static_cast <StartableQuiddity *>(user_data);
    return context->start ();
  }

  bool 
  StartableQuiddity::stop_wrapped (gpointer user_data)
  { 
    StartableQuiddity *context= static_cast <StartableQuiddity *>(user_data);
    return context->stop ();
  }
}//end of class

