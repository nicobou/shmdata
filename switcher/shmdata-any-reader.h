
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


#ifndef __SWITCHER_SHMDATA_ANY_READER_H__
#define __SWITCHER_SHMDATA_ANY_READER_H__

#include <functional>
#include <memory>
#include <string>
#include <shmdata/any-data-reader.h>
#include "json-builder.h"
#include "on-caps.h"

namespace switcher
{

  class ShmdataAnyReader : public OnCaps
  {
  public:
    typedef std::shared_ptr<ShmdataAnyReader> ptr;
    using Callback = std::function<void(void *, 
					int, 
					unsigned long long, 
					const char*, 
					void*)>;

    ShmdataAnyReader();
    ~ShmdataAnyReader();
    ShmdataAnyReader (const ShmdataAnyReader &) = delete;
    ShmdataAnyReader &operator= (const ShmdataAnyReader &) = delete;

    //conrfiguration member before starting:
    bool set_path (std::string path); //path needs to be fully specified
    bool set_callback (Callback cb, void* user_data);
    bool set_data_type (std::string data_type);
    bool set_absolute_timestamp (bool absolute_timestamp);

    //starting the reader:
    bool start ();

    //info + controls before and after starting the reader : 
    std::string get_path ();
    void mute (bool mute);
    bool is_muted ();
    JSONBuilder::Node get_json_root_node ();

  private:
    bool muted_ {false};
    std::string path_;
    Callback cb_ {nullptr};
    void* cb_user_data_ {nullptr};
    shmdata_any_reader_t *reader_ {nullptr};
    bool is_caps_set_ {false};

    JSONBuilder::ptr json_description_;
    void make_json_description ();
    static void on_data (shmdata_any_reader_t*, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
      const char* type_description, void* user_data);
  };
  
}  // end of namespace

#endif // ifndef
