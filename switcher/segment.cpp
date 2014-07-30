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

#include "segment.h"
#include "gst-utils.h"
#include "quiddity.h"
#include "scope-exit.h"

namespace switcher
{

  Segment::Segment() :
    shmdata_any_writers_ (),
    shmdata_any_readers_ (),
    shmdata_writers_ (),
    shmdata_readers_ (),
    shmdata_writers_description_ (new JSONBuilder()),
    shmdata_readers_description_ (new JSONBuilder()),
    segment_custom_props_ (new CustomPropertyHelper ()),
    json_writers_description_ (nullptr),
    json_readers_description_ (nullptr)
  {}

  Segment::~Segment()
  {}
  
  bool 
  Segment::init_segment (Quiddity *quid)
  {
    if (nullptr == quid)
      return false;
    quid_ = quid;

    update_shmdata_writers_description ();
    update_shmdata_readers_description ();

    //installing custom prop for json shmdata description
    json_writers_description_ = 
      segment_custom_props_->make_string_property ("shmdata-writers", 
						   "json formated shmdata writers description",
						   "",
						   (GParamFlags) G_PARAM_READABLE,
						   nullptr,
						   Segment::get_shmdata_writers_string,
						   this);
    
    json_readers_description_ = 
      segment_custom_props_->make_string_property ("shmdata-readers", 
						   "json formated shmdata readers description",
						   "",
						   (GParamFlags) G_PARAM_READABLE,
						   nullptr,
						   Segment::get_shmdata_readers_string,
						   this);
    
    quid_->install_property_by_pspec (segment_custom_props_->get_gobject (), 
				      json_writers_description_, 
				      "shmdata-writers",
				      "Shmdata Writers");
    quid_->install_property_by_pspec (segment_custom_props_->get_gobject (), 
				      json_readers_description_, 
				      "shmdata-readers",
				      "Shmdata Readers");
    
    return true;
  }
  
 
  bool 
  Segment::register_shmdata (ShmdataAnyWriter::ptr writer)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    std::unique_lock<std::mutex> lock (writers_mutex_);
    std::string name = writer->get_path ();
    if (name.empty () || 0 == name.compare (""))
      {
	g_warning ("Segment cannot register shmdata writer with no path");
	return false;
      }
    {//removing old one if present
      auto it = shmdata_any_writers_.find (name);
      if (shmdata_any_writers_.end () != it)
	shmdata_any_writers_.erase (name);
    }
 
    shmdata_any_writers_[name] = writer;

    {//JSON
      update_shmdata_writers_description ();
      segment_custom_props_->notify_property_changed (json_writers_description_);
      quid_->signal_emit ("on-new-shmdata-writer", 
      			  quid_->get_nick_name ().c_str (), 
      			  writer->get_path ().c_str (),
      			  (JSONBuilder::get_string (writer->get_json_root_node (), true)).c_str ());
    }
    return true;
  }

  bool 
  Segment::register_shmdata (ShmdataWriter::ptr writer)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    std::unique_lock<std::mutex> lock (writers_mutex_);
    std::string name = writer->get_path ();
    if (name.empty () || 0 == name.compare (""))
      {
	g_warning ("Segment cannot register shmdata writer with no path");
	return false;
      }
    {//removing old one if present
      auto it = shmdata_writers_.find (name);
      if (shmdata_writers_.end () != it)
	shmdata_writers_.erase (name);
    }
 
    shmdata_writers_[name] = writer;
 
   {//JSON
      update_shmdata_writers_description ();
      segment_custom_props_->notify_property_changed (json_writers_description_);
      quid_->signal_emit ("on-new-shmdata-writer", 
			  quid_->get_nick_name ().c_str (), 
			  writer->get_path ().c_str (),
			  (JSONBuilder::get_string (writer->get_json_root_node (), true)).c_str ());
    }
    return true;
  }

  
  bool Segment::register_shmdata (ShmdataReader::ptr reader)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    std::unique_lock<std::mutex> lock (readers_mutex_);
    std::string name = reader->get_path ();
    if (name.empty () || 0 == name.compare (""))
      {
	g_warning ("Segment cannot register shmdata writer with no path");
	return false;
      }
    {//removing old one if present
      auto it = shmdata_readers_.find (name);
      if (shmdata_readers_.end () != it)
	shmdata_readers_.erase (name);
    }

    shmdata_readers_[name] = reader;

    {//JSON
      update_shmdata_readers_description ();
      segment_custom_props_->notify_property_changed (json_readers_description_);
      quid_->signal_emit ("on-new-shmdata-reader", 
			  quid_->get_nick_name ().c_str (), 
			  reader->get_path ().c_str (),
			  JSONBuilder::get_string (reader->get_json_root_node (), true).c_str ());
    }
    return true;
  }

  bool Segment::register_shmdata (ShmdataAnyReader::ptr reader)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    std::unique_lock<std::mutex> lock (readers_mutex_);
    std::string name = reader->get_path ();
    if (name.empty () || 0 == name.compare (""))
      {
	g_warning ("Segment cannot register shmdata writer with no path");
	return false;
      }
    {//removing old one if present
      auto it = shmdata_any_readers_.find (name);
      if (shmdata_any_readers_.end () != it)
	shmdata_any_readers_.erase (name);
    }
    
    shmdata_any_readers_[name] = reader;

    {//JSON
      update_shmdata_readers_description ();
      segment_custom_props_->notify_property_changed (json_writers_description_);
      quid_->signal_emit ("on-new-shmdata-reader", 
      			  quid_->get_nick_name ().c_str (), 
      			  reader->get_path ().c_str (),
      			  JSONBuilder::get_string (reader->get_json_root_node (), true).c_str ());
    }
    return true;
  }

  bool Segment::unregister_shmdata (std::string shmdata_path)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    std::unique_lock<std::mutex> lock_w (writers_mutex_);
    std::unique_lock<std::mutex> lock_r (readers_mutex_);
    bool update_writer = false;
    bool update_reader = false;

    {//any reader
      auto it = shmdata_any_readers_.find (shmdata_path);
      if (shmdata_any_readers_.end () != it)
	{
	  shmdata_any_readers_.erase (it);
	  update_reader = true;
	}
    }

    {//reader
      auto it = shmdata_readers_.find (shmdata_path);
      if (shmdata_readers_.end () != it)
	{
	  shmdata_readers_.erase (it);
	  update_reader = true;
	}
    }
  
    if (update_reader)
      {
	update_shmdata_readers_description ();
	segment_custom_props_->notify_property_changed (json_readers_description_);
      }
    
      {//any writer
	auto it = shmdata_any_writers_.find (shmdata_path);
	if (shmdata_any_writers_.end () != it)
	  {
	    shmdata_any_writers_.erase (it);
	    update_writer = true;
	  }
      }
      
      {//writer
	auto it = shmdata_writers_.find (shmdata_path);
	if (shmdata_writers_.end () != it)
	  {
	    shmdata_writers_.erase (it);
	    update_writer = true;
	  }
      }
      
      if (update_writer)
	{
	  update_shmdata_writers_description ();
	  segment_custom_props_->notify_property_changed (json_writers_description_);
	}
    
    return update_reader || update_writer;
  }

  bool Segment::clear_shmdatas ()
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    std::unique_lock<std::mutex> lock_w (writers_mutex_);
    std::unique_lock<std::mutex> lock_r (readers_mutex_);
    
    bool update_writer_description = false;
    if (!shmdata_writers_.empty ())
      {
	shmdata_writers_.clear ();
	update_writer_description = true;
      }

    if (!shmdata_any_writers_.empty ())
      {
	shmdata_any_writers_.clear ();
	update_writer_description = true;
      }

    if (true == update_writer_description)
      {
	update_shmdata_writers_description ();
	segment_custom_props_->notify_property_changed (json_writers_description_);
      }

    if (!shmdata_readers_.empty ())
      {
	shmdata_readers_.clear ();
	update_shmdata_readers_description ();
	segment_custom_props_->notify_property_changed (json_readers_description_);
      }

    if (!shmdata_any_writers_.empty ())
      {
	shmdata_any_writers_.clear ();
	update_shmdata_writers_description ();
	segment_custom_props_->notify_property_changed (json_readers_description_);
      }

    if (!shmdata_any_readers_.empty ())
      {
	shmdata_any_readers_.clear ();
	update_shmdata_readers_description ();
	segment_custom_props_->notify_property_changed (json_readers_description_);
      }
    return true;
  }

  const gchar *
  Segment::get_shmdata_writers_string (void *user_data)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    Segment *context = static_cast<Segment *>(user_data);
    return context->writers_string_.c_str ();
  }

  const gchar *
  Segment::get_shmdata_readers_string (void *user_data)
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    Segment *context = static_cast<Segment *>(user_data);
    return context->readers_string_.c_str ();
  }

  void
  Segment::update_shmdata_writers_description ()
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};
    shmdata_writers_description_->reset();
    shmdata_writers_description_->begin_object ();
    shmdata_writers_description_->set_member_name ("shmdata_writers");
    shmdata_writers_description_->begin_array ();

    for (auto it : shmdata_writers_)
      shmdata_writers_description_->add_node_value (it.second->get_json_root_node ());

    for (auto it : shmdata_any_writers_)
      shmdata_writers_description_->add_node_value (it.second->get_json_root_node ());
    
    shmdata_writers_description_->end_array ();
    shmdata_writers_description_->end_object ();

    writers_string_ = shmdata_writers_description_->get_string (true);
  }

  void
  Segment::update_shmdata_readers_description ()
  {
    std::string func (__FUNCTION__); g_print ("entering %s\n", func.c_str ()); On_scope_exit {g_print ("leaving %s\n", func.c_str ());};

    shmdata_readers_description_->reset();
    shmdata_readers_description_->begin_object ();
    shmdata_readers_description_->set_member_name ("shmdata_readers");
    shmdata_readers_description_->begin_array ();

    for (auto &it : shmdata_readers_)
      shmdata_readers_description_->add_node_value (it.second->get_json_root_node ());
      
    for (auto &it : shmdata_any_readers_)
      shmdata_readers_description_->add_node_value (it.second->get_json_root_node ());

    shmdata_readers_description_->end_array ();
    shmdata_readers_description_->end_object ();

    readers_string_ = shmdata_readers_description_->get_string (true);
  }

}
