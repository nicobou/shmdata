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


#ifndef __SWITCHER_SHMDATA_READER_H__
#define __SWITCHER_SHMDATA_READER_H__

#include <tr1/memory>
#include <string>
#include <shmdata/base-reader.h>

namespace switcher
{

  class ShmdataReader
  {
  public:
    typedef std::tr1::shared_ptr<ShmdataReader> ptr;
    ShmdataReader();
    ~ShmdataReader();
    void plug (const char *socketName, GstElement *bin, GstElement *sink_element);
   
  private:
    std::string name_;
    shmdata_base_reader_t *reader_;
    GstElement *bin_;
    GstElement *sink_element_;
    static void on_first_data (shmdata_base_reader_t * context, void *user_data);
  };
  
}  // end of namespace

#endif // ifndef
