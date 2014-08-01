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

#include "shmdata-any-writer.h"
#include "shmdata-any-reader.h"
#include "shmdata-writer.h"
#include "shmdata-reader.h"
#include "counter-map.h"
#include "json-builder.h"
#include "custom-property-helper.h"
#include <vector>
#include <unordered_map>

namespace switcher
{
  class Quiddity;

  class Segment : public CounterMap 
  /*inherit from CounterMap for sharing counters between multiple DecodebinToShmdata*/
  {
  public:
    typedef std::shared_ptr<Segment> ptr;
    using OnConnect = std::function<bool(std::string)>;
    using OnDisconnect = std::function<bool(std::string)>;
    using OnDisconnectAll = std::function<bool()>;
    using CanSinkCaps = std::function<bool(std::string)>;

    Segment ();
    virtual ~Segment ();
    Segment (const Segment &) = delete;
    Segment &operator= (const Segment&) = delete;
    bool init_segment (Quiddity *quid);

  protected:
    bool register_shmdata (ShmdataWriter::ptr writer);
    bool register_shmdata (ShmdataAnyWriter::ptr writer);
    bool register_shmdata (ShmdataReader::ptr reader);
    bool register_shmdata (ShmdataAnyReader::ptr reader);
    bool unregister_shmdata (std::string shmdata_path);
    bool clear_shmdatas ();
    bool install_connect_method (OnConnect on_connect_cb,
				 OnDisconnect on_disconnect_cb,
				 OnDisconnectAll on_disconnect_all_cb,
				 CanSinkCaps on_can_sink_caps_cb,
				 uint max_reader);
      
  private:
    Quiddity *quid_ {nullptr};
    std::unordered_map <std::string, ShmdataAnyWriter::ptr> shmdata_any_writers_;
    std::unordered_map <std::string, ShmdataAnyReader::ptr> shmdata_any_readers_;
    std::unordered_map <std::string, ShmdataWriter::ptr> shmdata_writers_;
    std::unordered_map <std::string, ShmdataReader::ptr> shmdata_readers_;
    
    //reader methods to install by a subclass
    OnConnect on_connect_cb_ {nullptr};
    OnDisconnect on_disconnect_cb_ {nullptr};
    OnDisconnectAll on_disconnect_all_cb_ {nullptr};
    CanSinkCaps on_can_sink_caps_cb_ {nullptr};
    static gboolean connect_wrapped (gpointer path, gpointer user_data);
    static gboolean disconnect_wrapped (gpointer path, gpointer user_data);
    static gboolean disconnect_all_wrapped (gpointer /*unused*/, gpointer user_data);
    static gboolean can_sink_caps_wrapped (gpointer caps, gpointer user_data);

    //JSON
    JSONBuilder::ptr shmdata_writers_description_;
    std::mutex writers_mutex_ {}; //protecting from parallel registers
    std::string writers_string_ {};
    JSONBuilder::ptr shmdata_readers_description_;
    std::mutex readers_mutex_ {}; //protecting from parallel registers
    std::string readers_string_ {};
    CustomPropertyHelper::ptr segment_custom_props_;
    GParamSpec *json_writers_description_;
    GParamSpec *json_readers_description_;
    void update_shmdata_writers_description ();
    void update_shmdata_readers_description ();
    static const gchar *get_shmdata_writers_string (void *user_data);
    static const gchar *get_shmdata_readers_string (void *user_data);

    void populate_tree (std::string key, std::string caps);
  };
}  // end of namespace

#endif // ifndef
