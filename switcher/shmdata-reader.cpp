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

#include "shmdata-reader.h"
#include "gst-utils.h"

namespace switcher
{

  ShmdataReader::ShmdataReader() :
    connection_hook_ (NULL),
    hook_user_data_ (NULL),
    caps_ (NULL),
    path_ (),
    reader_ (shmdata_base_reader_new ()),
    bin_ (NULL),
    sink_element_ (NULL),
    funnel_ (NULL),
    g_main_context_ (NULL),
    elements_to_remove_ (),
    json_description_ (new JSONBuilder ()),
    start_mutex_ (),
    start_cond_ ()
  {}

  ShmdataReader::~ShmdataReader()
  {
    if (!path_.empty ())
      g_debug ("ShmdataReader: deleting %s", path_.c_str());
    else
      g_debug ("closing empty reader");
    shmdata_base_reader_close (reader_);
    if (!path_.empty ())
      g_debug ("ShmdataReader: %s deleted ", path_.c_str());
    else
      g_debug ("closing empty reader");
    if (NULL != caps_)
      gst_caps_unref (caps_);
  }

  void
  ShmdataReader::unlink_pad (GstPad * pad)
  {
    g_debug ("ShmdataReader::unlink_pad SHOULD NOT BE CALLED, ");
    GstPad *peer;
    if ((peer = gst_pad_get_peer (pad))) {
      if (gst_pad_get_direction (pad) == GST_PAD_SRC)
	gst_pad_unlink (pad, peer);
      else
	gst_pad_unlink (peer, pad);
      //checking if the pad has been requested and releasing it needed 
      GstPadTemplate *pad_templ = gst_pad_get_pad_template (peer);//check if this must be unrefed for GST 1
      if (GST_PAD_TEMPLATE_PRESENCE (pad_templ) == GST_PAD_REQUEST)
	gst_element_release_request_pad (gst_pad_get_parent_element(peer), peer);
      gst_object_unref (peer);
    }
  }

  std::string 
  ShmdataReader::get_path ()
  {
    return path_;
  }

  void 
  ShmdataReader::set_path (const char *absolute_path)
  {
    if (NULL == absolute_path)
      {
	g_debug ("shmdata path is NULL");
	return;
      }
    path_ = absolute_path;
    make_json_description ();
  }

  void 
  ShmdataReader::set_bin (GstElement *bin)
  {
    bin_ = bin;
  }

  void 
  ShmdataReader::set_g_main_context (GMainContext *context)
  {
    g_main_context_ = context;
  }


  void 
  ShmdataReader::set_sink_element (GstElement *sink_element)
  {   
    sink_element_ = sink_element;
  }

  void 
  ShmdataReader::start ()
  {
    // std::unique_lock<std::mutex> lock (start_mutex_); 
    // GstUtils::g_idle_add_full_with_context (g_main_context_,
    // 					    G_PRIORITY_DEFAULT_IDLE,
    // 					    start_idle,
    // 					    this,
    // 					    NULL);
    // g_debug ("%s: wait for start idle",
    // 	     __PRETTY_FUNCTION__);
    // start_cond_.wait (lock);
    // g_debug ("%s: start idle has unlocked",
    // 	     __PRETTY_FUNCTION__);
    start_idle (this);

  }

  gboolean
  ShmdataReader::start_idle (void *user_data)
  {
    ShmdataReader *context = static_cast<ShmdataReader *>(user_data);
    g_debug ("shmdata-reader::start_idle");
    shmdata_base_reader_close (context->reader_);
    GstUtils::clean_element (context->funnel_);
    context->reader_ = shmdata_base_reader_new ();
    shmdata_base_reader_set_g_main_context (context->reader_, context->g_main_context_);
    shmdata_base_reader_set_on_have_type_callback (context->reader_, 
						   ShmdataReader::on_have_type, 
						   context);
    if (context->path_.empty () ||  context->bin_ == NULL)
      {
	g_warning ("cannot start the shmdata reader: name or bin or sink element has not bin set");
	return FALSE;
      }
    shmdata_base_reader_set_callback (context->reader_, 
				      ShmdataReader::on_first_data, 
				      context);
    shmdata_base_reader_install_sync_handler (context->reader_, FALSE);
    shmdata_base_reader_set_bin (context->reader_, context->bin_);
    shmdata_base_reader_start (context->reader_, context->path_.c_str());
    g_debug ("shmdata-reader::start_idle done");
    // std::unique_lock<std::mutex> lock (context->start_mutex_);
    // context->start_cond_.notify_all ();
    return FALSE;//do not repeat
  }


  void 
  ShmdataReader::on_have_type (shmdata_base_reader_t *, 
			       GstCaps *caps, 
			       void *user_data)
  {
    if (NULL == user_data || NULL == caps)
      {
	g_warning ("ShmdataReader::on_have_type cannot save caps");
	return;
      }
    ShmdataReader *reader = static_cast<ShmdataReader *>(user_data);
    reader->caps_ = gst_caps_copy (caps);
  }

  void 
  ShmdataReader::stop ()
  {
    g_debug ("ShmdataReader::stop");
    shmdata_base_reader_close (reader_);
    if (NULL != caps_)
      {
	gst_caps_unref (caps_);
	caps_ = NULL;
      }
    GstUtils::clean_element (funnel_);
  } 
 
  void 
  ShmdataReader::set_on_first_data_hook (on_first_data_hook cb, void *user_data)
  {
    g_debug ("ShmdataReader::set_on_first_data_hook");
    connection_hook_ = cb;
    hook_user_data_ = user_data;
  }
 
  void 
  ShmdataReader::on_first_data (shmdata_base_reader_t *context, void *user_data)
  {
      ShmdataReader *reader = static_cast<ShmdataReader *>(user_data);
      g_debug (" ShmdataReader::on_first_data");
      if (reader->connection_hook_ != NULL) //user want to create the sink_element_ 
	reader->connection_hook_ (reader, reader->hook_user_data_);
      if (NULL != reader->sink_element_)
	if (!GST_IS_ELEMENT (GST_ELEMENT_PARENT (reader->sink_element_)))
	  gst_bin_add (GST_BIN (reader->bin_), reader->sink_element_);
      // else 
      // 	  g_debug ("ShmdataReader::on_first_data: (%s) sink element (%s) has a parent (%s) %d", 
      // 		      reader->get_path ().c_str(), 
      // 		      GST_ELEMENT_NAME (reader->sink_element_), 
      // 		      GST_ELEMENT_NAME(GST_ELEMENT_PARENT (reader->sink_element_)),
      // 		      GST_IS_ELEMENT(GST_ELEMENT_PARENT (reader->sink_element_)));
      GstUtils::make_element ("funnel", &reader->funnel_);
      gst_bin_add (GST_BIN (reader->bin_), reader->funnel_);
      gst_element_link (reader->funnel_, reader->sink_element_);
      GstUtils::sync_state_with_parent (reader->sink_element_);
      GstUtils::sync_state_with_parent (reader->funnel_);
      shmdata_base_reader_set_sink (context, reader->funnel_);
  }

  void
  ShmdataReader::make_json_description ()
  {
    json_description_->reset ();
    json_description_->begin_object ();
    json_description_->add_string_member ("path", path_.c_str ());
    json_description_->end_object ();
  }

  JSONBuilder::Node 
  ShmdataReader::get_json_root_node ()
  {
    return json_description_->get_root ();
  }

}
