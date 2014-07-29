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
  {
    //clear_shmdatas ();
  }
  
  bool 
  Segment::init_nico_segment (Quiddity *quid)
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
  
 
  void
  Segment::update_shmdata_writers_description ()
  {
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
  }

  void
  Segment::update_shmdata_readers_description ()
  {
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
  }

  bool 
  Segment::register_shmdata_any_writer (ShmdataAnyWriter::ptr writer)
  {
    const gchar *name = writer->get_path ().c_str ();
    if (g_strcmp0 (name, "") == 0)
      {
	g_warning ("Segment:: can not register shmdata writer with no path");
	return false;
      }
    shmdata_any_writers_[name] = writer;
    update_shmdata_writers_description ();
    segment_custom_props_->notify_property_changed (json_writers_description_);
    quid_->signal_emit ("on-new-shmdata-writer", 
			quid_->get_nick_name ().c_str (), 
			writer->get_path ().c_str (),
			(JSONBuilder::get_string (writer->get_json_root_node (), true)).c_str ());
    return true;
  }

  bool 
  Segment::register_shmdata_writer (ShmdataWriter::ptr writer)
  {
    const gchar *name = writer->get_path ().c_str ();
    if (g_strcmp0 (name, "") == 0)
      {
	g_warning ("Segment:: can not register shmdata writer with no path");
	return false;
      }
    shmdata_writers_[name] = writer;
    update_shmdata_writers_description ();
    segment_custom_props_->notify_property_changed (json_writers_description_);
    quid_->signal_emit ("on-new-shmdata-writer", 
			quid_->get_nick_name ().c_str (), 
			writer->get_path ().c_str (),
			(JSONBuilder::get_string (writer->get_json_root_node (), true)).c_str ());
    return true;
  }

  
  bool Segment::register_shmdata_reader (ShmdataReader::ptr reader)
  {
    const gchar *name = reader->get_path ().c_str ();
    if (g_strcmp0 (name, "") == 0)
      {
	g_warning ("Segment:: can not register shmdata reader with no path");
	return false;
      }
    shmdata_readers_[name] = reader;
    update_shmdata_readers_description ();
    segment_custom_props_->notify_property_changed (json_readers_description_);
    quid_->signal_emit ("on-new-shmdata-reader", 
			quid_->get_nick_name ().c_str (), 
			reader->get_path ().c_str (),
			JSONBuilder::get_string (reader->get_json_root_node (), true).c_str ());
    return true;
  }

  bool Segment::unregister_shmdata_reader (std::string shmdata_path)
  {
    auto it = shmdata_readers_.find (shmdata_path);
    if (shmdata_readers_.end () != it)
      shmdata_readers_.erase (it);
    update_shmdata_readers_description ();
    segment_custom_props_->notify_property_changed (json_readers_description_);
    return true;
  }

  bool Segment::register_shmdata_any_reader (ShmdataAnyReader::ptr reader)
  {
    const gchar *name = reader->get_path ().c_str ();
    if (g_strcmp0 (name, "") == 0)
      {
	g_warning ("Segment:: can not register shmdata reader with no path");
	return false;
      }
    shmdata_any_readers_[name] = reader;
    update_shmdata_readers_description ();
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_readers_description_);
    signal_emit ("on-new-shmdata-reader", 
		 get_nick_name ().c_str (), 
		 reader->get_path ().c_str (),
		 JSONBuilder::get_string (reader->get_json_root_node (), true).c_str ());
    return true;
  }

  bool Segment::unregister_shmdata_any_reader (std::string shmdata_path)
  {
    auto it = shmdata_any_readers_.find (shmdata_path);
    if (shmdata_any_readers_.end () != it)
      shmdata_any_readers_.erase (it);
    update_shmdata_readers_description ();
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_readers_description_);
    return true;
  }

  bool Segment::unregister_shmdata_any_writer (std::string shmdata_path)
  {
    auto it = shmdata_any_writers_.find (shmdata_path);
    if (shmdata_any_writers_.end () != it)
      shmdata_any_writers_.erase (it);
    update_shmdata_writers_description ();
    segment_custom_props_->notify_property_changed (json_writers_description_);
    return true;
  }

  bool Segment::unregister_shmdata_writer (std::string shmdata_path)
  {
    auto it = shmdata_writers_.find (shmdata_path);
    if (shmdata_writers_.end () != it)
      shmdata_writers_.erase (it);
    update_shmdata_writers_description ();
    segment_custom_props_->notify_property_changed (json_writers_description_);
    return true;
  }

  bool Segment::clear_shmdatas ()
  {
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
	GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_writers_description_);
      }

    if (!shmdata_any_readers_.empty ())
      {
	shmdata_any_readers_.clear ();
	update_shmdata_readers_description ();
	GObjectWrapper::notify_property_changed (gobject_->get_gobject (), json_readers_description_);
      }
    return true;
  }

  const gchar *
  Segment::get_shmdata_writers_string (void *user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    return context->shmdata_writers_description_->get_string (true).c_str ();
  }

  const gchar *
  Segment::get_shmdata_readers_string (void *user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    return context->shmdata_readers_description_->get_string (true).c_str ();
  }

}
