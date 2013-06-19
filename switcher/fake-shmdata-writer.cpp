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

#include "fake-shmdata-writer.h"
#include "gst-utils.h"

namespace switcher
{

  QuiddityDocumentation FakeShmdataWriter::doc_ ("fake source", "fakeshmsrc",
						 "fake existing shmdata writer");
  
  
  bool
  FakeShmdataWriter::init ()
  {
    set_name (gst_element_get_name (bin_));

    //registering add_data_stream
    register_method("add_shmdata_path",
		    (void *)&add_shmdata_path_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("add_shmdata_path", 
			    "add an existing shmdata writer path", 
			    Method::make_arg_description ("name", 
							  "the shmdata writer path",
							  NULL));
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
  
  
  QuiddityDocumentation 
  FakeShmdataWriter::get_documentation ()
  {
    return doc_;
  }
  
}
