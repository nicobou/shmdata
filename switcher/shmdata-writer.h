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


#ifndef __SWITCHER_SHMDATA_WRITER_H__
#define __SWITCHER_SHMDATA_WRITER_H__

#include <tr1/memory>
#include <string>
#include <shmdata/base-writer.h>

namespace switcher
{

  class ShmdataWriter
  {
  public:
    typedef std::tr1::shared_ptr<ShmdataWriter> ptr;
    ShmdataWriter();
    ~ShmdataWriter();
    bool set_name (std::string name); 
    bool set_absolute_name (std::string name); //path will be fully specified
    void plug (GstElement *bin, GstElement *source_element,GstCaps *caps);//caps does not need to be fully specified
    void plug (GstElement *bin, GstPad *source_pad);
  private:
    std::string name_;
    shmdata_base_writer_t *writer_;
    GstElement *tee_;
    GstElement *queue_;
    GstElement *fakesink_;
  };
  
}  // end of namespace

#endif // ifndef
