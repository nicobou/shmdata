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

#include "switcher/base-entity.h"
#include "switcher/runtime.h"
#include "switcher/string-map.h"
#include "switcher/connector.h"
#include <memory>
#include <vector>

namespace switcher
{

  class Segment : public BaseEntity
  {
  public:
    typedef std::tr1::shared_ptr<Segment> ptr;
    Segment();
    // the segment is managing itself the presence/attachment with the runtime
    void set_runtime (Runtime *runtime);
    GstElement *get_bin ();

    //TODO register this function as char * get_connectors () returning json
    std::vector<std::string> get_src_connectors ();

    Connector::ptr get_connector (std::string name);

    bool connect (char *src_connector_name, Segment *segment);

    //wrappers for calls from base entity manager
    static void set_runtime_wrapped (gpointer runtime, gpointer context);
    static gboolean connect_wrapped (gpointer connector_name, gpointer segment, gpointer user_data);
   
    
  protected:
    GstElement *bin_;
    Runtime *runtime_;
    StringMap<Connector::ptr> connectors_;
  };
  
}  // end of namespace

#endif // ifndef
