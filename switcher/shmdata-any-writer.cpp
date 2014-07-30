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

#include "shmdata-any-writer.h"
#include "gst-utils.h"

namespace switcher
{

  ShmdataAnyWriter::ShmdataAnyWriter() :
    started_ (false),
    path_ (),
    writer_ (shmdata_any_writer_init ()),
    json_description_ (new JSONBuilder()),
    thread_safe_ ()    
  {
    shmdata_any_writer_set_debug (writer_, SHMDATA_ENABLE_DEBUG);
  }

  ShmdataAnyWriter::~ShmdataAnyWriter()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    //g_print ("%s\n",__FUNCTION__);
    shmdata_any_writer_close (writer_);
    if (!path_.empty ())
      g_debug ("ShmdataAnyWriter: %s deleted", path_.c_str());
  }

  //WARNING if the file exist it will be deleted
  bool 
  ShmdataAnyWriter::set_path (std::string name)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    //g_print ("%s\n",__FUNCTION__);
    GFile *shmfile = g_file_new_for_commandline_arg (name.c_str());
    if( g_file_query_exists (shmfile, nullptr))
      {    
	//thrash it
	g_debug ("ShmdataAnyWriter::set_path warning: file %s exists and will be deleted.",name.c_str());
	if (! g_file_delete (shmfile, nullptr, nullptr)) 
	  {
	    g_debug ("ShmdataAnyWriter::set_path error: file %s is already existing and cannot be trashed.",name.c_str());
	    return false;
	  }
      }
    
    return set_path_without_deleting (name);
  }

  bool 
  ShmdataAnyWriter::set_path_without_deleting (std::string name)
  {
    //g_print ("%s\n",__FUNCTION__);
    //setting the writer
    shmdata_any_writer_set_path (writer_,name.c_str());
    path_ = name;
    make_json_description ();
    return true;
  }

  std::string 
  ShmdataAnyWriter::get_path ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    //g_print ("%s\n",__FUNCTION__);
    return path_;
  }

  void 
  ShmdataAnyWriter::start ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    shmdata_any_writer_start (writer_);
    started_ = true;
  }

  void
  ShmdataAnyWriter::set_data_type (std::string data_type)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    shmdata_any_writer_set_data_type (writer_, data_type.c_str ());
    set_negociated_caps (std::move(data_type));
  }

  void
  ShmdataAnyWriter::push_data (void *data, 
			       size_t data_size, 
			       unsigned long long clock,
			       void (*data_not_required_anymore) (void *),
			       void *user_data)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    ////g_print ("%s\n",__FUNCTION__);
    if (started_)
      shmdata_any_writer_push_data (writer_,
				    data,
				    data_size,
				    clock,
				    data_not_required_anymore, 
				    user_data);
  }

  void
  ShmdataAnyWriter::push_data_auto_clock (void *data, 
					  size_t data_size, 
					  void (*data_not_required_anymore) (void *),
					  void *user_data)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    ////g_print ("%s\n",__FUNCTION__);
    if (started_)
      shmdata_any_writer_push_data (writer_,
				    data,
				    data_size,
				    clock_.get_count (),
				    data_not_required_anymore, 
				    user_data);
  }
  
  void
  ShmdataAnyWriter::make_json_description ()
  {
    //g_print ("%s\n",__FUNCTION__);
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("path", path_.c_str ());
    json_description_->end_object ();
  }

  JSONBuilder::Node 
  ShmdataAnyWriter::get_json_root_node ()
  {
    //g_print ("%s\n",__FUNCTION__);
    return json_description_->get_root ();
  }

  bool 
  ShmdataAnyWriter::started ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    return started_;
  }

}
