/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/shmdata-writer.h"

namespace switcher
{

  ShmdataWriter::ShmdataWriter() :
    writer_ (shmdata_base_writer_init ())
  {}
  
  bool 
  ShmdataWriter::set_name (std::string name)
  {
    g_printerr ("ShmdataWriter::set_name (const char *name) IS NOT IMPLEMENTED");
    return false;
  }
  
  bool 
  ShmdataWriter::set_absolute_name (std::string name)
  {
    //this might be entered by the user so not deleting it
    if(shmdata_base_writer_set_path (writer_,name.c_str()) == SHMDATA_FILE_EXISTS)
      {
	g_printerr ("**** The file %s exists, therefore a shmdata cannot be operated with this path.\n", name.c_str());
	return false;
      }
    else
      {
	name_ = name;
	return true;
      }
  }
  
  void 
  ShmdataWriter::plug (GstElement *pipeline, GstElement *source_element)
  {
    shmdata_base_writer_plug (writer_, pipeline, source_element);
  }
  
}
