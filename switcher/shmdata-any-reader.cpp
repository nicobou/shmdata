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

#include "shmdata-any-reader.h"

namespace switcher
{

  ShmdataAnyReader::ShmdataAnyReader() :
    started_ (false),
    path_ (""),
    reader_ (shmdata_any_reader_init ()),
    json_description_ (new JSONBuilder())
  {
    shmdata_any_reader_set_debug (reader_, SHMDATA_ENABLE_DEBUG);
    shmdata_any_reader_set_on_data_handler(reader_, ShmdataAnyReader::on_data, this);
  }

  ShmdataAnyReader::~ShmdataAnyReader()
  {
    shmdata_any_reader_close (reader_);
  }

  bool 
  ShmdataAnyReader::set_path (std::string name)
  {
    if (started_ == true)
      shmdata_any_reader_close(reader_);
    path_ = name;
    make_json_description();
    return true;
  }

  std::string 
  ShmdataAnyReader::get_path ()
  {
    return path_;
  }

  bool
  ShmdataAnyReader::set_callback (Callback cb, void* user_data)
  {
    cb_user_data_ = user_data;
    cb_ = cb;
    return true;
  }

  void 
  ShmdataAnyReader::start ()
  {
    if (cb_ == nullptr)
      return;
    shmdata_any_reader_start(reader_, path_.c_str());
    started_ = true;
  }

  void
  ShmdataAnyReader::stop()
  {
    shmdata_any_reader_close(reader_);
    started_ = false;
  }

  void
  ShmdataAnyReader::make_json_description ()
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("path", path_.c_str ());
    json_description_->end_object ();
  }

  JSONBuilder::Node 
  ShmdataAnyReader::get_json_root_node ()
  {
    return json_description_->get_root ();
  }

  bool 
  ShmdataAnyReader::started ()
  {
    return started_;
  }
  
  void
  ShmdataAnyReader::on_data (shmdata_any_reader_t*, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
    const char* type_description, void* user_data)
  {
    ShmdataAnyReader* ctx = (ShmdataAnyReader*)user_data;
    ctx->cb_(data, data_size, timestamp, type_description, ctx->cb_user_data_);
    shmdata_any_reader_free(shmbuf);
  }
}
