
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

namespace switcher
{

  class ShmdataAnyReader
  {
  public:
    typedef std::shared_ptr<ShmdataAnyReader> ptr;
    typedef std::function<void(void*, int, unsigned long long, const char*, void*)> Callback;

    ShmdataAnyReader();
    ~ShmdataAnyReader();
    ShmdataAnyReader (const ShmdataAnyReader &) = delete;
    ShmdataAnyReader &operator= (const ShmdataAnyReader &) = delete;
    bool set_path (std::string name); //path needs to be fully specified
    std::string get_path ();
    bool set_callback (Callback cb, void* user_data);
    void start ();
    void stop ();
    bool started ();

    //get json doc:
    JSONBuilder::Node get_json_root_node ();

  private:
    bool started_;
    std::string path_;

    Callback cb_ {nullptr};
    void* cb_user_data_ {nullptr};

    shmdata_any_reader_t *reader_ {nullptr};
    JSONBuilder::ptr json_description_;
    void make_json_description ();

    static void on_data (shmdata_any_reader_t*, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
      const char* type_description, void* user_data);
  };
  
}  // end of namespace

#endif // ifndef
