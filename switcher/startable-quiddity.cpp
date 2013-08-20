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

  }

  void
  StartableQuiddity::init_startable (Quiddity *quid)
  {
    quid->publish_method ("Start",
     			  "start", 
     			  "start processing", 
     			  "success or fail",
     			  Method::make_arg_description ("none",
     							NULL),
     			  (Method::method_ptr) &StartableQuiddity::start_wrapped, 
     			  G_TYPE_BOOLEAN,
     			  Method::make_arg_type_description (G_TYPE_NONE, NULL),
     			  this);
     quid->publish_method ("Stop",
			   "stop", 
			   "stop processing", 
			   "success or fail",
			   Method::make_arg_description ("none",
							 NULL),
			   (Method::method_ptr) &StartableQuiddity::stop_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_NONE, NULL),
			   this);
  }
  
   gboolean 
   StartableQuiddity::start_wrapped (gpointer unused, gpointer user_data)
   {
     StartableQuiddity *context= static_cast <StartableQuiddity *>(user_data);

     if (!context->start ())
       return FALSE;
     return TRUE;
   }

   gboolean 
   StartableQuiddity::stop_wrapped (gpointer unused, gpointer user_data)
   { 
     StartableQuiddity *context= static_cast <StartableQuiddity *>(user_data);
     if (!context->stop ())
       return FALSE;
     return TRUE;
   }
}//end of class

