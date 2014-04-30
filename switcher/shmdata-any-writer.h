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


#ifndef __SWITCHER_SHMDATA_ANY_WRITER_H__
#define __SWITCHER_SHMDATA_ANY_WRITER_H__

#include <memory>
#include <string>
#include <shmdata/any-data-writer.h>
#include "json-builder.h"

namespace switcher
{

  class ShmdataAnyWriter
  {
  public:
    typedef std::shared_ptr<ShmdataAnyWriter> ptr;
    ShmdataAnyWriter();
    ~ShmdataAnyWriter();
    ShmdataAnyWriter (const ShmdataAnyWriter &) = delete;
    ShmdataAnyWriter &operator= (const ShmdataAnyWriter &) = delete;
    bool set_path (std::string name); //path needs to be fully specified
    std::string get_path ();
    void set_data_type (std::string data_type);
    void push_data (void *data, 
		    size_t data_size, 
		    unsigned long long clock,
		    void (*data_not_required_anymore) (void *),
		    void *user_data);
    void start ();
    bool started ();

    //get json doc:
    JSONBuilder::Node get_json_root_node ();

  private:
    bool started_;
    std::string path_;
    shmdata_any_writer_t *writer_;
    JSONBuilder::ptr json_description_;
    void make_json_description ();
    bool set_path_without_deleting (std::string name);
    /* static void on_handoff_cb (GstElement* object, */
    /* 				GstBuffer* buf, */
    /* 				GstPad* pad, */
    /* 				gpointer user_data); */
  };
  
}  // end of namespace

#endif // ifndef
