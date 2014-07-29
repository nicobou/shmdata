/*
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


#ifndef __SWITCHER_SEGMENT_H__
#define __SWITCHER_SEGMENT_H__

//#include "quiddity.h"
//#include "gpipe.h"
#include "shmdata-any-writer.h"
#include "shmdata-any-reader.h"
#include "shmdata-writer.h"
#include "shmdata-reader.h"
#include "counter-map.h"
#include "json-builder.h"
#include "custom-property-helper.h"
#include <vector>
#include <unordered_map>

//FIXME separate shmdata management and runtime

namespace switcher
{
  class Quiddity;

  class Segment : public CounterMap 
  /*inherit from CounterMap for sharing counters between multiple DecodebinToShmdata*/
  {
  public:
    typedef std::shared_ptr<Segment> ptr;
    Segment ();
    virtual ~Segment ();
    Segment (const Segment &) = delete;
    Segment &operator= (const Segment&) = delete;

    bool init_segment (Quiddity *quid);

  protected:
    bool register_shmdata_writer (ShmdataWriter::ptr writer);
    bool unregister_shmdata_writer (std::string shmdata_path);
    bool register_shmdata_any_writer (ShmdataAnyWriter::ptr writer);
    bool unregister_shmdata_any_writer (std::string shmdata_path);
    bool register_shmdata_reader (ShmdataReader::ptr reader);
    bool unregister_shmdata_reader (std::string shmdata_path);
    bool register_shmdata_any_reader (ShmdataAnyReader::ptr reader);
    bool unregister_shmdata_any_reader (std::string shmdata_path);
    bool clear_shmdatas ();

  private:
    Quiddity *quid_ {nullptr};
    std::unordered_map <std::string, ShmdataAnyWriter::ptr> shmdata_any_writers_;
    std::unordered_map <std::string, ShmdataAnyReader::ptr> shmdata_any_readers_;
    std::unordered_map <std::string, ShmdataWriter::ptr> shmdata_writers_;
    std::unordered_map <std::string, ShmdataReader::ptr> shmdata_readers_;
    JSONBuilder::ptr shmdata_writers_description_;
    JSONBuilder::ptr shmdata_readers_description_;
    //shmdatas as param
    CustomPropertyHelper::ptr segment_custom_props_;
    GParamSpec *json_writers_description_;
    GParamSpec *json_readers_description_;

    void update_shmdata_writers_description ();
    void update_shmdata_readers_description ();
    static const gchar *get_shmdata_writers_string (void *user_data);
    static const gchar *get_shmdata_readers_string (void *user_data);
  };
}  // end of namespace

#endif // ifndef
