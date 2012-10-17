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


#ifndef __SWITCHER_SEGMENT_H__
#define __SWITCHER_SEGMENT_H__

#include "switcher/quiddity.h"
#include "switcher/runtime.h"
#include "switcher/string-map.h"
#include "switcher/shmdata-writer.h"
#include "switcher/shmdata-reader.h"
#include <memory>
#include <vector>

namespace switcher
{

  class Segment : public Quiddity
  {
  public:
    typedef std::tr1::shared_ptr<Segment> ptr;
    Segment ();
    ~Segment ();
    // the segment is managing itself the presence/attachment with the runtime
    void set_runtime (Runtime::ptr runtime);

    //TODO rename and register this function returning json
    std::vector<std::string> get_src_connectors ();

    //wrappers for calls from base quiddity manager
    static void set_runtime_wrapped (gpointer runtime, gpointer context);
    
  protected:
    GstElement *get_bin ();
    GstElement *bin_;
    Runtime::ptr runtime_;
    StringMap<ShmdataWriter::ptr> shmdata_writers_;
    StringMap<ShmdataReader::ptr> shmdata_readers_;


  };
  
}  // end of namespace

#endif // ifndef
