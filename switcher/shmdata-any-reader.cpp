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
    path_ (),
    reader_ (shmdata_any_reader_init ()),
    json_description_ (new JSONBuilder())
  {}

  ShmdataAnyReader::~ShmdataAnyReader()
  {
    shmdata_any_reader_close (reader_);
  }

  bool 
  ShmdataAnyReader::set_path (std::string path)
  {
    shmdata_any_reader_set_debug (reader_, SHMDATA_ENABLE_DEBUG);
    shmdata_any_reader_set_on_data_handler(reader_, ShmdataAnyReader::on_data, this);
    path_ = path;
    make_json_description();
    return true;
  }

  bool 
  ShmdataAnyReader::start ()
  {
    if (path_.empty ())
      return false;
    shmdata_any_reader_start(reader_, path_.c_str());
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
  ShmdataAnyReader::mute (bool mute)
  { 
    muted_ = mute; 
  }

  bool
  ShmdataAnyReader::is_muted ()
  {
    return muted_;
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

  void
  ShmdataAnyReader::on_data (shmdata_any_reader_t*, 
			     void* shmbuf, 
			     void* data, 
			     int data_size, 
			     unsigned long long timestamp,
			     const char *type_description, 
			     void* user_data)
  {
    ShmdataAnyReader* context = static_cast <ShmdataAnyReader *>(user_data);
    if (!context->is_caps_set_)
      {
	context->set_negociated_caps (std::string (type_description));
	context->is_caps_set_ = true;
      }
    if (nullptr != context->cb_ && !context->muted_)
      context->cb_(data, data_size, timestamp, type_description, context->cb_user_data_);
    shmdata_any_reader_free (shmbuf);
  }
  
  bool 
  ShmdataAnyReader::set_data_type (std::string data_type)
  {
    shmdata_any_reader_set_data_type (reader_,
				      data_type.c_str ());
    return true;
  }
  
  bool 
  ShmdataAnyReader::set_absolute_timestamp (bool absolute_timestamp)
  {
    if (absolute_timestamp)
      shmdata_any_reader_set_absolute_timestamp (reader_,
						 SHMDATA_ENABLE_ABSOLUTE_TIMESTAMP);
    else
      shmdata_any_reader_set_absolute_timestamp (reader_,
						 SHMDATA_DISABLE_ABSOLUTE_TIMESTAMP);
    
    return true;
  }

}
