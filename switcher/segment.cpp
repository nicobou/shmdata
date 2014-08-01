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

    writer->set_on_caps (std::bind (&Segment::populate_tree, 
    				    this, 
    				    std::string (".shmdata.writer.") + name,
    				    std::placeholders::_1));

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
 
    writer->set_on_caps (std::bind (&Segment::populate_tree, 
    				    this, 
    				    std::string (".shmdata.writer.") + name,
    				    std::placeholders::_1));
    
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
    
    reader->set_on_caps (std::bind (&Segment::populate_tree, 
    				    this, 
    				    std::string (".shmdata.reader.") + name,
    				    std::placeholders::_1));

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

    reader->set_on_caps (std::bind (&Segment::populate_tree, 
    				    this, 
    				    std::string (".shmdata.reader.") + name,
    				    std::placeholders::_1));
    
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
    Segment *context = static_cast<Segment *>(user_data);
    return context->writers_string_.c_str ();
  }

  const gchar *
  Segment::get_shmdata_readers_string (void *user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    return context->readers_string_.c_str ();
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

    writers_string_ = shmdata_writers_description_->get_string (true);
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

    readers_string_ = shmdata_readers_description_->get_string (true);
  }

  void 
  Segment::populate_tree (std::string key, std::string caps)
  {
    std::string category;
    std::string mime_type (caps.begin (), std::find (caps.begin (), caps.end (), (',')));
    
    if (std::string::npos != mime_type.find ("video/x-raw"))
      category = "video";
    else if (std::string::npos != mime_type.find ("video/x-"))
      category = "compressed video";
    else if (std::string::npos != mime_type.find ("audio/midi"))
      category = "midi";
    else if (std::string::npos != mime_type.find ("audio/x-raw"))
      category = "audio";
    else if (std::string::npos != mime_type.find ("audio/x-"))
      category = "compressed audio";
    else if (std::string::npos != mime_type.find ("application/x-libloserialized-osc"))
      category = "osc";
    else if (std::string::npos != mime_type.find ("application/x-"))
      {
	auto it = std::find (mime_type.begin(), 
			     mime_type.end (), 
			     '-');
	it ++;
	if (mime_type.end () == it)
	  category = "unknown";
	else
	  category = std::string (it,
				  mime_type.end ());
      }
    else
      category = mime_type;

    data::Tree::ptr tree = data::make_tree ();
    tree->graft (".category", data::make_tree (category));
    tree->graft (".caps", data::make_tree (caps));
    //attaching it to the quiddity (at the root) 
    quid_->graft_tree (key, tree);
  }
 
  bool 
  Segment::install_connect_method (OnConnect on_connect_cb,
				   OnDisconnect on_disconnect_cb,
				   OnDisconnectAll on_disconnect_all_cb,
				   CanSinkCaps on_can_sink_caps_cb,
				   uint max_reader)
  {
    if (quid_ == nullptr)
    {
        g_warning("Segment::%s - Segment not initialized yet, call Segment::init_segment(this) first", __FUNCTION__);
        return false;
    }

    data::Tree::ptr tree = data::make_tree ();
    tree->graft (".max_reader", data::make_tree (max_reader));
    quid_->graft_tree (".shmdata", tree);

    on_connect_cb_ = on_connect_cb;
    on_disconnect_cb_ = on_disconnect_cb;
    on_disconnect_all_cb_ = on_disconnect_all_cb; 
    on_can_sink_caps_cb_ = on_can_sink_caps_cb;

    quid_->install_method ("Connect",
			   "connect",
			   "connect to a shmdata", 
			   "success or fail",
			   Method::make_arg_description ("Shmdata Path",
							 "path",
							 "shmdata path to connect with",
							 nullptr),
			   (Method::method_ptr)&Segment::connect_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_STRING, nullptr),
			   this);

    quid_->install_method ("Disconnect",
			   "disconnect",
			   "disconnect a shmdata", 
			   "success or fail",
			   Method::make_arg_description ("Shmdata Path",
							 "path",
							 "shmdata path to connect with",
							 nullptr),
			   (Method::method_ptr)&Segment::disconnect_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_STRING, nullptr),
			   this);
    
    quid_->install_method ("Disconnect All",
			   "disconnect-all",
			   "disconnect all shmdata reader", 
			   "success or fail",
			   Method::make_arg_description ("none",
							 nullptr),
			   (Method::method_ptr)&Segment::disconnect_all_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_NONE, nullptr),
			   this);

    quid_->install_method ("Can sink caps",
			   "can-sink-caps",
			   "can we connect with this caps", 
			   "true or false",
			   Method::make_arg_description ("String Caps",
							 "caps",
							 "caps as a string",
							 nullptr),
			   (Method::method_ptr)&Segment::can_sink_caps_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_STRING, nullptr),
			   this);

    return true;
  }
    
  gboolean
  Segment::connect_wrapped (gpointer path, gpointer user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    
    if (nullptr == context->on_connect_cb_)
      {
	g_warning ("on connect callback not installed\n");
	return FALSE;
      }
    
    if (context->on_connect_cb_ ((char *)path))
      return TRUE;
    else
      return FALSE;
  }

  gboolean
  Segment::disconnect_wrapped (gpointer path, gpointer user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    
    if (nullptr == context->on_disconnect_cb_)
      {
  	g_warning ("on disconnect callback not installed\n");
  	return FALSE;
      }
    
    if (context->on_disconnect_cb_ ((char *)path))
      return TRUE;
    else
      return FALSE;
  }

  gboolean
  Segment::disconnect_all_wrapped (gpointer /*unused*/, gpointer user_data)
  {
    Segment* context = static_cast<Segment*>(user_data);
    On_scope_exit {context->clear_shmdatas ();};

    if (nullptr == context->on_disconnect_all_cb_)
      {
  	g_warning ("on disconnect all callback not installed\n");
  	return FALSE;
      }
    
    if (context->on_disconnect_all_cb_ ())
      return TRUE;
    else
      return FALSE;
  }

  gboolean
  Segment::can_sink_caps_wrapped (gpointer caps, gpointer user_data)
  {
    Segment *context = static_cast<Segment *>(user_data);
    
    if (nullptr == context->on_can_sink_caps_cb_)
      {
  	g_warning ("on disconnect callback not installed\n");
  	return FALSE;
      }
    
    if (context->on_can_sink_caps_cb_ ((char *)caps))
      return TRUE;
    else
      return FALSE;
  }
  
}
