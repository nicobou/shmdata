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
 * The Runtime class
 */

#include "runtime.h"
#include "quiddity.h" 
#include "quiddity-command.h" 
#include "custom-property-helper.h"
#include "gst-utils.h"
#include <shmdata/base-reader.h>
#include <gst/interfaces/xoverlay.h>

namespace switcher
{

  Runtime::Runtime () :
    pipeline_ (gst_pipeline_new (NULL)),
    speed_ (1.0),
    position_tracking_source_ (NULL),
    source_funcs_ (),
    source_ (NULL),
    quid_ (NULL),
    custom_props_ (new CustomPropertyHelper ()),
    play_pause_spec_ (NULL),
    play_ (true),
    seek_spec_ (NULL),
    seek_ (0.0),
    length_ (0)
  {}

  void
  Runtime::init_runtime (Quiddity &quiddity)
  {
    quid_ = &quiddity;
    source_funcs_.prepare = source_prepare;
    source_funcs_.check = source_check;
    source_funcs_.dispatch = source_dispatch;
    source_funcs_.finalize = source_finalize;
    source_ = g_source_new (&source_funcs_, sizeof (GstBusSource));
    ((GstBusSource*)source_)->bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline_));
    g_source_set_callback(source_, (GSourceFunc)bus_called, NULL, NULL);
    if (NULL == quid_->get_g_main_context ())
      g_warning ("%s: g_main_context is NULL",
     		 __FUNCTION__);
    g_source_attach(source_, quid_->get_g_main_context ());
    gst_bus_set_sync_handler (((GstBusSource*)source_)->bus, bus_sync_handler, this); 
    g_source_unref (source_);
    ((GstBusSource*)source_)->inited = FALSE;
    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
    //GstUtils::wait_state_changed (pipeline_);
    play_pause_spec_ = 
      custom_props_->make_boolean_property ("play", 
					    "play",
					    (gboolean)TRUE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Runtime::set_play,
					    Runtime::get_play,
					    this);
    seek_spec_ = 
      custom_props_->make_double_property ("seek", 
					   "seek (in percent)",
					   0.0,
					   1.0,
					   0.0,
					   (GParamFlags) G_PARAM_READWRITE,
					   Runtime::set_seek,
					   Runtime::get_seek,
					   this);

  }

  void
  Runtime::install_play_pause ()
  {
    quid_->install_property_by_pspec (custom_props_->get_gobject (), 
				      play_pause_spec_, 
				      "play",
				      "Play");
  }
  
  void 
  Runtime::install_seek ()
  {
    quid_->install_property_by_pspec (custom_props_->get_gobject (), 
				      seek_spec_, 
				      "seek",
				      "Seek");
  }
  
  void
  Runtime::install_speed ()
  {
    quid_->install_method ("Speed",
			   "speed", 
			   "controle speed of runtime", 
			   "success or fail",
			   Method::make_arg_description ("Speed",
							 "speed",
							 "1.0 is normal speed, 0.5 is half the speed and 2.0 is double speed",
							 NULL),
			   (Method::method_ptr) &speed_wrapped, 
			   G_TYPE_BOOLEAN,
			   Method::make_arg_type_description (G_TYPE_DOUBLE, NULL),
			   this);
  }
  
  Runtime::~Runtime ()
  {
    if (position_tracking_source_ != NULL)
       g_source_destroy (position_tracking_source_);
    GstUtils::clean_element (pipeline_);
    if (!g_source_is_destroyed (source_))
      g_source_destroy (source_);
  }


  void 
  Runtime::play (gboolean play)
  {
    if (play_ == play)
      return;
    play_ = play;
    if (NULL == position_tracking_source_
	&& NULL != quid_->get_g_main_context ())
      {
	guint position_tracking_id = 
	  GstUtils::g_timeout_add_to_context (200, 
					      (GSourceFunc) query_position, 
					      this,
					      quid_->get_g_main_context ());
	position_tracking_source_ = 
	  g_main_context_find_source_by_id (quid_->get_g_main_context (),
					    position_tracking_id);
      }
    
    if (TRUE == play)
      gst_element_set_state (pipeline_, 
			     GST_STATE_PLAYING);
    else
      gst_element_set_state (pipeline_, 
			     GST_STATE_PAUSED);
    custom_props_->notify_property_changed (play_pause_spec_);
  }

  void 
  Runtime::set_play (gboolean play, void *user_data)
  {
    Runtime *context = static_cast<Runtime *> (user_data);
    context->play (play);
  }

  gboolean 
  Runtime::get_play (void *user_data)
  {
    Runtime *context = static_cast<Runtime *> (user_data);
    return context->play_;
  }

  bool
  Runtime::seek (gdouble position)
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

    custom_props_->notify_property_changed (seek_spec_);
    if (!ret)
      g_debug ("seek not handled\n");
    return true;
  }
  
  gdouble 
  Runtime::get_seek (void *user_data)
  {
    Runtime *context = static_cast<Runtime *> (user_data);
    return context->seek_;
  }
  void 
  Runtime::set_seek (gdouble position, void *user_data)
  {
    Runtime *context = static_cast<Runtime *> (user_data);
    context->seek (position);
  }

  gboolean
  Runtime::speed_wrapped (gdouble speed, gpointer user_data)
  {
    Runtime *context = static_cast<Runtime *>(user_data);
      
    g_debug ("speed_wrapped %f", speed);

    if (context->speed (speed))
      return TRUE;
    else
      return FALSE;
  }

  bool
  Runtime::speed (gdouble speed)
  {
    g_debug ("Runtime::speed %f", speed);

    speed_ = speed;
    
    GstQuery *query;
    gboolean res;

    //query position
    query = gst_query_new_position (GST_FORMAT_TIME);
res = gst_element_query (pipeline_, query);
    gint64 cur_pos = 0;
    if (res) {
      gst_query_parse_position (query, 
				NULL, 
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
  Runtime::query_position_and_length ()
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
  Runtime::query_position (gpointer user_data)
  {
    Runtime *context = static_cast<Runtime *>(user_data);
    context->query_position_and_length ();
    /* call me again */
    return TRUE;
  }
  
  gboolean
  Runtime::run_command (gpointer user_data)
  {
    QuidCommandArg *context = static_cast<QuidCommandArg *>(user_data);
    QuiddityManager_Impl::ptr manager = context->self->quid_->manager_impl_.lock ();
    if ((bool) manager && context->command != NULL)
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
			       NULL, //do not care of return value
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
      g_warning ("Runtime::bus_sync_handler, cannot run command");
    
    delete context;
    return FALSE; //do not repeat run_command
  }  

  GstElement * 
  Runtime::get_pipeline ()
  {
    return pipeline_;
  }

  void
  Runtime::print_one_tag (const GstTagList * list, const gchar * tag, gpointer user_data)
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
  Runtime::bus_sync_handler (GstBus */*bus*/,
			     GstMessage *msg, 
			     gpointer user_data) 
  {
    shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), 
										 "shmdata_base_reader");
    Runtime *context = static_cast<Runtime *>(user_data);
    if ( reader != NULL)
      {
	if (NULL != msg && shmdata_base_reader_process_error (reader, msg)) 
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
	g_debug ("Runtime::bus_sync_handler Error: %s from %s", error->message, GST_MESSAGE_SRC_NAME(msg));
	
	QuiddityCommand *command = (QuiddityCommand *) g_object_get_data (G_OBJECT (msg->src), 
									  "on-error-command");
	//removing command in order to get it invoked once
	g_object_set_data (G_OBJECT (msg->src), 
			   "on-error-command",
			   (gpointer)NULL);

	if (command != NULL)
	  {
	    g_debug ("error contains data (on-error-command) ");
	    QuidCommandArg *args = new QuidCommandArg ();
	    args->self = context;
	    args->command = command;
	    if (command->time_ > 1)
	      GstUtils::g_timeout_add_to_context ((guint) command->time_,
						  (GSourceFunc)run_command,
						  args,
						  context->quid_->get_g_main_context ());   
	    else
	      GstUtils::g_idle_add_full_with_context (context->quid_->get_g_main_context (),
						      G_PRIORITY_DEFAULT_IDLE,
						      (GSourceFunc) run_command,   
						      (gpointer)args,
						      NULL);   

	  }

	g_error_free (error);
	return GST_BUS_DROP; 
      }

    if (NULL != msg->structure)
      if (gst_structure_has_name (msg->structure, "prepare-xwindow-id"))
	{
	  guintptr *window_handle = (guintptr *)g_object_get_data (G_OBJECT (msg->src), 
								   "window-handle");
	  if (window_handle != NULL)
	    gst_x_overlay_set_window_handle (GST_X_OVERLAY (msg->src), *window_handle);
	}
    
    if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_TAG)
      {
	GstTagList *tags = NULL;
	gst_message_parse_tag (msg, &tags);
	// g_print ("Got tags from element %s:\n", GST_OBJECT_NAME (msg->src));
	// gst_tag_list_foreach (tags, print_one_tag, NULL);
	// g_print ("\n");
	gst_tag_list_free (tags);
      }
    return GST_BUS_PASS; 
  }

  gboolean
  Runtime::bus_called (GstBus */*bus*/,
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
          // gst_message_parse_state_changed (msg, &old_state, &new_state, NULL);
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
  Runtime::source_prepare(GSource *source, gint *timeout)
  {
    GstBusSource *bsrc = (GstBusSource *)source;
    *timeout = -1;
    return gst_bus_have_pending (bsrc->bus);
  }
  
  gboolean 
  Runtime::source_check(GSource *source)
  {
    GstBusSource *bsrc = (GstBusSource *)source;
    return gst_bus_have_pending (bsrc->bus);
  }


  gboolean 
  Runtime::source_dispatch(GSource *source, GSourceFunc callback,
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
  Runtime::source_finalize (GSource * source)
  {
    GstBusSource *bsrc = (GstBusSource *)source;
    gst_object_unref (bsrc->bus);
    bsrc->bus = NULL;
  }
  

}

