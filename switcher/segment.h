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

#include "quiddity.h"
#include "runtime.h"
#include "shmdata-writer.h"
#include "shmdata-reader.h"
#include "json-builder.h"
#include <vector>
#include <unordered_map>

namespace switcher
{
  class Segment : public Quiddity, public Runtime
  {
  public:
    typedef std::shared_ptr<Segment> ptr;
    Segment ();
    virtual ~Segment ();
    Segment (const Segment &) = delete;
    Segment &operator= (const Segment&) = delete;
    virtual bool init_segment () = 0;
    bool init ();

  protected:
    GstElement *get_bin ();
    GstElement *bin_; //FIXME should be private
    bool register_shmdata_writer (ShmdataWriter::ptr writer);
    bool unregister_shmdata_writer (std::string shmdata_path);
    bool register_shmdata_reader (ShmdataReader::ptr reader);
    bool unregister_shmdata_reader (std::string shmdata_path);
    bool clear_shmdatas ();
    bool reset_bin ();

  private:
    std::unordered_map <std::string, ShmdataWriter::ptr> shmdata_writers_;
    std::unordered_map <std::string, ShmdataReader::ptr> shmdata_readers_;
    JSONBuilder::ptr shmdata_writers_description_;
    JSONBuilder::ptr shmdata_readers_description_;
    //shmdatas as param
    static GParamSpec *json_writers_description_;
    static GParamSpec *json_readers_description_;

    void make_bin ();
    void clean_bin ();
    void update_shmdata_writers_description ();
    void update_shmdata_readers_description ();
    static bool get_shmdata_writers_by_gvalue (GValue *value, void *user_data);
    static bool get_shmdata_readers_by_gvalue (GValue *value, void *user_data);
    static gboolean clean_element_invoke (gpointer user_data);
  };
}  // end of namespace

#endif // ifndef
