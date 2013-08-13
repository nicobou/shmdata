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

#include "fake-shmdata-writer.h"
#include "gst-utils.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(FakeShmdataWriter,
				       "Shmdata From Software",
				       "fake source", 
				       "add a shmdata from an other software",
				       "LGPL",
				       "fakeshmsrc",
				       "Nicolas Bouillot");
  
  
  bool
  FakeShmdataWriter::init ()
  {
    set_name (gst_element_get_name (bin_));

    publish_method ("Add External Shmdata",
		    "add_shmdata_path",
		    "add an existing shmdata writer path",
		    "success or fail",
		    Method::make_arg_description ("Shmdata Path",
						  "name", 
						  "a shmdata writer path",
						  NULL),
		    (Method::method_ptr) &add_shmdata_path_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    true,
		    true,
		    this);
    return true;
  }
  
  gboolean
  FakeShmdataWriter::add_shmdata_path_wrapped (gpointer name, 
						 gpointer user_data)
  {
    FakeShmdataWriter *context = static_cast<FakeShmdataWriter *>(user_data);
  
    if (context->add_shmdata_path ((char *)name))
      return TRUE;
    else
      return FALSE;
  }

  bool
  FakeShmdataWriter::add_shmdata_path (std::string name)
  {
    //creating a shmdata
    ShmdataWriter::ptr connector;
    connector.reset (new ShmdataWriter ());
    connector->set_path_without_deleting (name.c_str());
    register_shmdata_writer (connector);

    g_message ("%s created a new shmdata writer (%s)", 
     	       get_nick_name ().c_str(), 
     	       name.c_str ());
    return true;
  }
  
}
