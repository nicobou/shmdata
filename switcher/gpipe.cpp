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

/**
 * The GPipe class
 */

#include "gpipe.h"
#include "quiddity.h" 
#include "quiddity-command.h" 
#include "custom-property-helper.h"
#include "gst-utils.h"
#include <shmdata/base-reader.h>
#include <gst/interfaces/xoverlay.h>
#include <algorithm>

namespace switcher
{

  GPipe::GPipe () :
    pipeline_ (gst_pipeline_new (nullptr)),
    source_funcs_ (),
    gpipe_custom_props_ (new CustomPropertyHelper ())
  {
    make_bin ();
    init_segment (this);
  }

  GPipe::~GPipe ()
  {
    if (!commands_.empty ())
      for (auto &it : commands_)
	if (!g_source_is_destroyed (it))
	  g_source_destroy (it);
    if (position_tracking_source_ != nullptr)
       g_source_destroy (position_tracking_source_);
    GstUtils::clean_element (pipeline_);
    if (!g_source_is_destroyed (source_))
      g_source_destroy (source_);
  }

  bool
  GPipe::init ()
  {
    source_funcs_.prepare = source_prepare;
    source_funcs_.check = source_check;
    source_funcs_.dispatch = source_dispatch;
    source_funcs_.finalize = source_finalize;
    source_ = g_source_new (&source_funcs_, sizeof (GstBusSource));
    ((GstBusSource*)source_)->bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline_));
    g_source_set_callback(source_, (GSourceFunc)bus_called, nullptr, nullptr);
    if (nullptr == get_g_main_context ())
      {
	g_warning ("%s: g_main_context is nullptr",
     		 __FUNCTION__);
	return false;
      }
    g_source_attach(source_, get_g_main_context ());
    gst_bus_set_sync_handler (((GstBusSource*)source_)->bus, bus_sync_handler, this); 
    g_source_unref (source_);
    ((GstBusSource*)source_)->inited = FALSE;
    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
    //GstUtils::wait_state_changed (pipeline_);
    play_pause_spec_ = 
      gpipe_custom_props_->make_boolean_property ("play", 
					    "play",
					    (gboolean)TRUE,
					    (GParamFlags) G_PARAM_READWRITE,
					    GPipe::set_play,
					    GPipe::get_play,
					    this);
    seek_spec_ = 
      gpipe_custom_props_->make_double_property ("seek", 
					   "seek (in percent)",
					   0.0,
					   1.0,
					   0.0,
					   (GParamFlags) G_PARAM_READWRITE,
					   GPipe::set_seek,
					   GPipe::get_seek,
					   this);
    return init_gpipe ();
  }

  void
  GPipe::install_play_pause ()
  {
    install_property_by_pspec (gpipe_custom_props_->get_gobject (), 
				      play_pause_spec_, 
				      "play",
				      "Play");
  }
  
  void 
  GPipe::install_seek ()
  {
    install_property_by_pspec (gpipe_custom_props_->get_gobject (), 
				      seek_spec_, 
				      "seek",
				      "Seek");
  }
  
  void
  GPipe::install_speed ()
  {
    install_method ("Speed",
			   "speed", 
			   "controle speed of pipeline", 
			   "success or fail",
			   Method::make_arg_description ("Speed",
							 "speed",
							 "1.0 is normal speed, 0.5 is half the speed and 2.0 is double speed",
							 nullptr),
			   (Method::method_ptr) &speed_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_DOUBLE, nullptr),
			   this);
  }
  
  void 
  GPipe::play (gboolean play)
  {
    if (play_ == play)
      return;
    play_ = play;
    if (nullptr == position_tracking_source_
	&& nullptr != get_g_main_context ())
      position_tracking_source_ = 
	GstUtils::g_timeout_add_to_context (200, 
					    (GSourceFunc) query_position, 
					    this,
					    get_g_main_context ());
    if (TRUE == play)
      gst_element_set_state (pipeline_, 
			     GST_STATE_PLAYING);
    else
      gst_element_set_state (pipeline_, 
			     GST_STATE_PAUSED);
    gpipe_custom_props_->notify_property_changed (play_pause_spec_);
  }

  void 
  GPipe::set_play (gboolean play, void *user_data)
  {
    GPipe *context = static_cast<GPipe *> (user_data);
    context->play (play);
  }

  gboolean 
  GPipe::get_play (void *user_data)
  {
    GPipe *context = static_cast<GPipe *> (user_data);
    return context->play_;
  }

  bool
  GPipe::seek (gdouble position)
  {
    gboolean ret = FALSE;
    ret = gst_element_seek (pipeline_,  
			    speed_,  
			    GST_FORMAT_TIME,  
			    (GstSeekFlags)(//GST_SEEK_FLAG_FLUSH | 
					   GST_SEEK_FLAG_ACCURATE), 
			    //| GST_SEEK_FLAG_SKIP 
			    //| GST_SEEK_FLAG_KEY_UNIT, //using key unit is breaking synchronization 
			    GST_SEEK_TYPE_SET,  
			    position * length_ * GST_MSECOND,
			    GST_SEEK_TYPE_NONE,  
			    GST_CLOCK_TIME_NONE);  

    gpipe_custom_props_->notify_property_changed (seek_spec_);
    if (!ret)
      g_debug ("seek not handled\n");
    return true;
  }
  
  gdouble 
  GPipe::get_seek (void *user_data)
  {
    GPipe *context = static_cast<GPipe *> (user_data);
    return context->seek_;
  }
  void 
  GPipe::set_seek (gdouble position, void *user_data)
  {
    GPipe *context = static_cast<GPipe *> (user_data);
    context->seek (position);
  }

  gboolean
  GPipe::speed_wrapped (gdouble speed, gpointer user_data)
  {
    GPipe *context = static_cast<GPipe *>(user_data);
      
    g_debug ("speed_wrapped %f", speed);

    if (context->speed (speed))
      return TRUE;
    else
      return FALSE;
  }

  bool
  GPipe::speed (gdouble speed)
  {
    g_debug ("GPipe::speed %f", speed);

    speed_ = speed;
    
    GstQuery *query;
    gboolean res;

    //query position
    query = gst_query_new_position (GST_FORMAT_TIME);
res = gst_element_query (pipeline_, query);
    gint64 cur_pos = 0;
    if (res) {
      gst_query_parse_position (query, 
				nullptr, 
				&cur_pos);
      
      g_debug ("cur pos = %" GST_TIME_FORMAT"\n", 
	       GST_TIME_ARGS (cur_pos));
    }
    else {
      g_warning ("position query failed...");
    }
    gst_query_unref (query);
  
    gboolean ret;
    ret = gst_element_seek (pipeline_,  
			    speed,  
			    GST_FORMAT_TIME,  
			    (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | 
					   GST_SEEK_FLAG_ACCURATE), 
			    GST_SEEK_TYPE_SET,  
			    cur_pos,  
			    GST_SEEK_TYPE_NONE,  
			    GST_CLOCK_TIME_NONE);  
    
    if (!ret)
      g_debug ("speed not handled\n");

    return true;
  }

  void
  GPipe::query_position_and_length ()
  {
    GstFormat fmt = GST_FORMAT_TIME;
    gint64 pos;
    
    if (gst_element_query_position (pipeline_, &fmt, &pos)
     	&& gst_element_query_duration (pipeline_, &fmt, &length_)) 
      {
	 // g_print ("Time: %" GST_TIME_FORMAT " / %" GST_TIME_FORMAT "\r",
	 // 	 GST_TIME_ARGS (pos), GST_TIME_ARGS (length_));
      }
    
  }
  
  gboolean
  GPipe::query_position (gpointer user_data)
  {
    GPipe *context = static_cast<GPipe *>(user_data);
    context->query_position_and_length ();
    /* call me again */
    return TRUE;
  }
  
  gboolean
  GPipe::run_command (gpointer user_data)
  {
    QuidCommandArg *context = static_cast<QuidCommandArg *>(user_data);
    QuiddityManager_Impl::ptr manager = context->self->manager_impl_.lock ();
    if ((bool) manager && context->command != nullptr)
      {
	switch (context->command->id_)
	  {
	  case QuiddityCommand::remove:
	    manager->remove (context->command->args_[0]);
	    break;
	  case QuiddityCommand::invoke:
	    {
	      manager->invoke (context->command->args_[0], 
			       context->command->args_[1], 
			       nullptr, //do not care of return value
			       context->command->vector_arg_);
	    }
	    break;
	  case QuiddityCommand::set_property:
	    {
	      manager->set_property (context->command->args_[0], 
				     context->command->args_[1], 
				     context->command->vector_arg_[0]);
	    }
	    break;

	  default:
	    g_debug ("on-error-command: %s not implemented\n", 
	 	     QuiddityCommand::get_string_from_id(context->command->id_));
	  }
      }
    else
      g_warning ("GPipe::bus_sync_handler, cannot run command");
    
    auto it = std::find (context->self->commands_.begin (),
			 context->self->commands_.end (),
			 context->src);
    if (context->self->commands_.end () != it)
      context->self->commands_.erase (it);
    delete context;
    return FALSE; //do not repeat run_command
  }  

  GstElement * 
  GPipe::get_pipeline ()
  {
    return pipeline_;
  }

  void
  GPipe::print_one_tag (const GstTagList * list, const gchar * tag, gpointer user_data)
  {
     // int i, num;
    
     // num = gst_tag_list_get_tag_size (list, tag);
     // for (i = 0; i < num; ++i) {
     //   const GValue *val;
      
     //   /* Note: when looking for specific tags, use the g_tag_list_get_xyz() API,
     //    * we only use the GValue approach here because it is more generic */
     //   val = gst_tag_list_get_value_index (list, tag, i);
     //   if (G_VALUE_HOLDS_STRING (val)) {
     // 	g_print ("\t%20s : %s\n", tag, g_value_get_string (val));
     //   } else if (G_VALUE_HOLDS_UINT (val)) {
     // 	g_print ("\t%20s : %u\n", tag, g_value_get_uint (val));
     //   } else if (G_VALUE_HOLDS_DOUBLE (val)) {
     // 	g_print ("\t%20s : %g\n", tag, g_value_get_double (val));
     //   } else if (G_VALUE_HOLDS_BOOLEAN (val)) {
     // 	g_print ("\t%20s : %s\n", tag,
     // 		 (g_value_get_boolean (val)) ? "true" : "false");
     //   } else if (GST_VALUE_HOLDS_BUFFER (val)) {
     // 	g_print ("\t%20s : buffer of size %u\n", tag,
     // 		 GST_BUFFER_SIZE (gst_value_get_buffer (val)));
     //   } else if (GST_VALUE_HOLDS_DATE (val)) {
     // 	g_print ("\t%20s : date (year=%u,...)\n", tag,
     // 		 g_date_get_year (gst_value_get_date (val)));
     //   } else {
     // 	g_print ("\t%20s : tag of type '%s'\n", tag, G_VALUE_TYPE_NAME (val));
     //   }
     // }
  }
  
  GstBusSyncReply 
  GPipe::bus_sync_handler (GstBus */*bus*/,
			     GstMessage *msg, 
			     gpointer user_data) 
  {
    shmdata_base_reader_t *reader = 
      (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), 
						   "shmdata_base_reader");
    GPipe *context = static_cast<GPipe *>(user_data);

    // g_print ("-----------%s-----%s--------------------------\n",
    // 	     G_OBJECT_TYPE_NAME(G_OBJECT (msg->src)),
    // 	     GST_MESSAGE_TYPE_NAME (msg));

    if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_QOS)
      {
	GstFormat format;
	guint64 processed;
	guint64 dropped;
	gst_message_parse_qos_stats (msg,
				     &format,
				     &processed,
				     &dropped);
	// g_print ("QOS from %s, format %d, processed %lu dropped %lu\n",
	// 	 G_OBJECT_TYPE_NAME(G_OBJECT (msg->src)),
	// 	 format,
	// 	 processed,
	// 	 dropped);
	return GST_BUS_PASS; 
      }

    if (reader != nullptr)
      {
	if (nullptr != msg && shmdata_base_reader_process_error (reader, msg)) 
	  return GST_BUS_DROP; 
	else 
	  return GST_BUS_PASS; 
      }

    if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_ERROR)
      {
	gchar *debug;
	GError *error;
	
	gst_message_parse_error (msg, &error, &debug);
	g_free (debug);
	g_debug ("GPipe::bus_sync_handler Error: %s from %s", error->message, GST_MESSAGE_SRC_NAME(msg));
	
	QuiddityCommand *command = (QuiddityCommand *) g_object_get_data (G_OBJECT (msg->src), 
									  "on-error-command");
	//removing command in order to get it invoked once
	g_object_set_data (G_OBJECT (msg->src), 
			   "on-error-command",
			   (gpointer)nullptr);

	if (command != nullptr)
	  {
	    g_debug ("error contains data (on-error-command) ");
	    QuidCommandArg *args = new QuidCommandArg ();
	    args->self = context;
	    args->command = command;
	    args->src = nullptr;
	    if (command->time_ > 1)
	      {
		args->src = g_timeout_source_new ((guint) command->time_);
		g_source_set_callback (args->src, 
				       (GSourceFunc)run_command, 
				       args, 
				       nullptr);
		context->commands_.push_back (args->src);
		g_source_attach (args->src, context->get_g_main_context ());   
		g_source_unref(args->src);
	      }
	    else
	      {
		GstUtils::g_idle_add_full_with_context (context->get_g_main_context (),
							G_PRIORITY_DEFAULT_IDLE,
							(GSourceFunc) run_command,   
							(gpointer) args,
							nullptr);
	      }   

	  }

	g_error_free (error);
	return GST_BUS_DROP; 
      }

    if (nullptr != msg->structure)
      if (gst_structure_has_name (msg->structure, "prepare-xwindow-id"))
	{
	  guintptr *window_handle = (guintptr *)g_object_get_data (G_OBJECT (msg->src), 
								   "window-handle");
	  if (window_handle != nullptr)
	    gst_x_overlay_set_window_handle (GST_X_OVERLAY (msg->src), *window_handle);
	}
    
    if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_TAG)
      {
	// GstTagList *tags = nullptr;
	// gst_message_parse_tag (msg, &tags);
	// g_print ("Got tags from element %s:\n", GST_OBJECT_NAME (msg->src));
	// gst_tag_list_foreach (tags, print_one_tag, nullptr);
	// g_print ("\n");
	// gst_tag_list_free (tags);
      }
    return GST_BUS_PASS; 
  }

  gboolean
  GPipe::bus_called (GstBus */*bus*/,
		       GstMessage *msg,
		       gpointer /*user_data*/)
  {
    switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_debug ("bus_call End of stream, name: %s",
	       GST_MESSAGE_SRC_NAME(msg));
      break;
    case GST_MESSAGE_SEGMENT_DONE:
      g_debug ("bus_call segment done");
      break;
    case GST_MESSAGE_ERROR: 
      gchar *debug;
      GError *error;
      
      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);
      g_debug ("bus_call Error: %s from %s", error->message, GST_MESSAGE_SRC_NAME(msg));
      g_error_free (error);
      return FALSE;

      break;
    case GST_MESSAGE_STATE_CHANGED:
          // GstState old_state, new_state;
          // gst_message_parse_state_changed (msg, &old_state, &new_state, nullptr);
          // g_debug ("Element %s changed state from %s to %s.",
          // 	       GST_OBJECT_NAME (msg->src),
          // 	       gst_element_state_get_name (old_state),
          // 	       gst_element_state_get_name (new_state));
          // if (GST_IS_ELEMENT (GST_ELEMENT_PARENT (msg->src)))
          // 	{
          // 	  g_debug ("parent :%s (%s)",
          // 		   GST_OBJECT_NAME (GST_ELEMENT_PARENT (msg->src)),
          // 		   gst_element_state_get_name (GST_STATE(GST_ELEMENT_PARENT (msg->src))));
          // 	}
      break;
    default:
      //g_debug ("message %s from %s",GST_MESSAGE_TYPE_NAME(msg),GST_MESSAGE_SRC_NAME(msg));
      break;
    }
    return TRUE;
  }
  
  gboolean 
  GPipe::source_prepare(GSource *source, gint *timeout)
  {
    GstBusSource *bsrc = (GstBusSource *)source;
    *timeout = -1;
    return gst_bus_have_pending (bsrc->bus);
  }
  
  gboolean 
  GPipe::source_check(GSource *source)
  {
    GstBusSource *bsrc = (GstBusSource *)source;
    return gst_bus_have_pending (bsrc->bus);
  }


  gboolean 
  GPipe::source_dispatch(GSource *source, GSourceFunc callback,
			   gpointer user_data)
  {
    GstBusFunc   handler = (GstBusFunc) callback;
    GstBusSource *bsrc = (GstBusSource *) source;
    gboolean     result = FALSE;
    
    if (handler)
      {
        GstMessage *message = gst_bus_pop (bsrc->bus);
        if (message)
	  {
            result = handler(bsrc->bus, message, user_data);
            gst_message_unref (message);
	  }
      }
    
    return result;
  }
  
  void 
  GPipe::source_finalize (GSource * source)
  {
    GstBusSource *bsrc = (GstBusSource *)source;
    gst_object_unref (bsrc->bus);
    bsrc->bus = nullptr;
  }

 void 
  GPipe::make_bin ()
  {
    GstUtils::make_element ("bin", &bin_);
    g_object_set (G_OBJECT (bin_), "async-handling",TRUE, nullptr);
    gst_bin_add (GST_BIN (get_pipeline ()), bin_);
    GstUtils::wait_state_changed (get_pipeline ());
    GstUtils::sync_state_with_parent (bin_);
    GstUtils::wait_state_changed (bin_);
  }
  
  void
  GPipe::clean_bin()
  {
    g_debug ("GPipe, bin state %s, target %s, num children %d ", 
	     gst_element_state_get_name (GST_STATE (bin_)), 
	     gst_element_state_get_name (GST_STATE_TARGET (bin_)), 
	     GST_BIN_NUMCHILDREN(GST_BIN (bin_)));
    
    GstUtils::wait_state_changed (bin_);
    
    if (GST_IS_ELEMENT (bin_))
      {
	//FIXME clear_shmdatas ();

	g_debug ("GPipe, bin state %s, target %s, num children %d ", 
		 gst_element_state_get_name (GST_STATE (bin_)), 
		 gst_element_state_get_name (GST_STATE_TARGET (bin_)), 
		 GST_BIN_NUMCHILDREN(GST_BIN (bin_)));
	
	if (g_list_length (GST_BIN_CHILDREN (bin_)) > 0)
	  {
	    GList *child = nullptr, *children = GST_BIN_CHILDREN (bin_);
	    for (child = children; child != nullptr; child = g_list_next (child)) 
	      {
		g_debug ("segment warning: child %s", GST_ELEMENT_NAME (GST_ELEMENT (child->data)));
		//GstUtils::clean_element (GST_ELEMENT (child->data));
	      }
	  }
	g_debug ("going to clean bin_");
	GstUtils::clean_element (bin_);
	g_debug ("GPipe: cleaning internal bin");
      }
  }

  GstElement *
  GPipe::get_bin()
  {
    return bin_;
  }
  
  bool 
  GPipe::reset_bin ()
  {
    clean_bin ();
    make_bin ();
    return true;
 }

}

