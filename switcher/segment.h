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


#ifndef __SWITCHER_SEGMENT_H__
#define __SWITCHER_SEGMENT_H__

#include "quiddity.h"
#include "runtime.h"
#include "string-map.h"
#include "shmdata-writer.h"
#include "shmdata-reader.h"
#include "json-builder.h"
#include <memory>
#include <vector>

namespace switcher
{

  class Segment : public Quiddity
  {
  public:
    typedef std::shared_ptr<Segment> ptr;
    Segment ();
    ~Segment ();
    // the segment is managing itself the presence/attachment with the runtime
    void set_runtime (Runtime::ptr runtime);

    //wrappers for calls from base quiddity manager
    static void set_runtime_wrapped (gpointer runtime, gpointer context);
    
    static bool get_shmdata_writers_by_gvalue (GValue *value, void *user_data);
    static bool get_shmdata_readers_by_gvalue (GValue *value, void *user_data);

  protected:
    GstElement *get_bin ();
    GstElement *bin_;
    Runtime::ptr runtime_;
    bool register_shmdata_writer (ShmdataWriter::ptr writer);
    bool unregister_shmdata_writer (std::string shmdata_path);
    bool register_shmdata_reader (ShmdataReader::ptr reader);
    bool unregister_shmdata_reader (std::string shmdata_path);
    bool clear_shmdatas ();
    bool reset_bin ();

  private:
    void make_bin ();
    void clean_bin ();
    StringMap<ShmdataWriter::ptr> shmdata_writers_;
    StringMap<ShmdataReader::ptr> shmdata_readers_;
    JSONBuilder::ptr shmdata_writers_description_;
    JSONBuilder::ptr shmdata_readers_description_;
    void update_shmdata_writers_description ();
    void update_shmdata_readers_description ();
    //shmdatas as param
    static GParamSpec *json_writers_description_;
    static GParamSpec *json_readers_description_;
  };
  
}  // end of namespace

#endif // ifndef
