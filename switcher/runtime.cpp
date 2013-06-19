/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * The Runtime class
 */

#include "runtime.h"
#include "gst-utils.h"
#include <shmdata/base-reader.h>

namespace switcher
{

  QuiddityDocumentation Runtime::doc_ ("runtime", "runtime",
					     "Complete pipeline container and scheduler");

  bool
  Runtime::init ()
  {
    speed_ = 1.0;
    pipeline_ = gst_pipeline_new (NULL);
    set_name (gst_element_get_name (pipeline_));
    
     source_funcs_ = {
       source_prepare,
       source_check,
       source_dispatch,
       source_finalize
     };
     source_ = g_source_new (&source_funcs_, sizeof (GstBusSource));

     ((GstBusSource*)source_)->bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline_));
     g_source_set_callback(source_, (GSourceFunc)bus_called, NULL, NULL);

     GMainContext *g_main_context = get_g_main_context ();
     if (g_main_context == NULL)
       return FALSE;
     g_source_attach(source_, g_main_context);
     gst_bus_set_sync_handler (((GstBusSource*)source_)->bus, bus_sync_handler, this); 
     g_source_unref (source_);
     ((GstBusSource*)source_)->inited = FALSE;

     // GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline_)); 
     // gst_bus_add_watch (bus, bus_called, NULL);
    
     // gst_bus_set_sync_handler (bus, bus_sync_handler, this);  
     // gst_object_unref (bus); 
    
    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
    GstUtils::wait_state_changed (pipeline_);

    //registering play
    register_method("play",
		    (void *)&play_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"play", 
			    (char *)"activate the runtime", 
			    Method::make_arg_description ("none",
							  NULL));

    //registering pause
    register_method("pause",
		    (void *)&pause_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"pause", 
			    (char *)"pause the runtime", 
			    Method::make_arg_description ((char *)"none",
							  NULL));

     //registering seek
    register_method("seek",
		    (void *)&seek_wrapped, 
		    Method::make_arg_type_description (G_TYPE_DOUBLE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"seek", 
			    (char *)"seek the runtime", 
			    Method::make_arg_description ((char *)"position",
							  (char *)"position in milliseconds",
							  NULL));

    //registering speed
    register_method("speed",
		    (void *)&speed_wrapped, 
		    Method::make_arg_type_description (G_TYPE_DOUBLE, NULL),
		    (gpointer)this);
    set_method_description ((char *)"speed", 
			    (char *)"controle speed of runtime", 
			    Method::make_arg_description ((char *)"speed",
							  (char *)"1.0 is normal speed, 0.5 is half the speed and 2.0 is double speed",
							  NULL));

    return true;
  }
  
  Runtime::~Runtime ()
  {
     gst_element_set_state (pipeline_, GST_STATE_NULL);
     gst_object_unref (GST_OBJECT (pipeline_));
     if (!g_source_is_destroyed (source_))
       g_source_destroy (source_);
  }

  gboolean
  Runtime::play_wrapped (gpointer unused, gpointer user_data)
  {
    Runtime *context = static_cast<Runtime *>(user_data);
      
    if (context->play ())
      return TRUE;
    else
      return FALSE;
  }
  
  bool
  Runtime::play ()
  {
    g_debug ("Runtime::play");
    gst_element_set_state (pipeline_, GST_STATE_PLAYING);
    return true;
  }
  

  gboolean
  Runtime::pause_wrapped (gpointer unused, gpointer user_data)
  {
    Runtime *context = static_cast<Runtime *>(user_data);
      
    if (context->pause ())
      return TRUE;
    else
      return FALSE;
  }

  bool
  Runtime::pause ()
  {
    g_debug ("Runtime::pause");
    gst_element_set_state (pipeline_, GST_STATE_PAUSED);
    return true;
  }
  
  gboolean
  Runtime::seek_wrapped (gdouble position, gpointer user_data)
  {
    Runtime *context = static_cast<Runtime *>(user_data);
      
    g_debug ("seek_wrapped %f", position);

    if (context->seek (position))
      return TRUE;
    else
      return FALSE;
  }

  bool
  Runtime::seek (gdouble position)
  {
    g_debug ("Runtime::seek %f", position);
    // GstQuery *query;
    // gboolean res;
    // query = gst_query_new_segment (GST_FORMAT_TIME);
    // res = gst_element_query (pipeline_, query);
    // gdouble rate = -2.0;
    // gint64 start_value = -2.0;
    // gint64 stop_value = -2.0;
    // if (res) {
    //   gst_query_parse_segment (query, &rate, NULL, &start_value, &stop_value);
    //   g_debug ("rate = %f start = %" GST_TIME_FORMAT" stop = %" GST_TIME_FORMAT"\n", 
    // 	       rate,
    // 	       GST_TIME_ARGS (start_value),
    // 	       GST_TIME_ARGS (stop_value));
    // }
    // else {
    //   g_warning ("duration query failed...");
    // }
    // gst_query_unref (query);
  
    gboolean ret;
    ret = gst_element_seek (pipeline_,  
			    speed_,  
			    GST_FORMAT_TIME,  
			    (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | 
					   GST_SEEK_FLAG_ACCURATE), 
			    //| GST_SEEK_FLAG_SKIP 
			    //| GST_SEEK_FLAG_KEY_UNIT, //using key unit is breaking synchronization 
			    GST_SEEK_TYPE_SET,  
			    position * GST_MSECOND,  
			    GST_SEEK_TYPE_NONE,  
			    GST_CLOCK_TIME_NONE);  
    
    if (!ret)
     g_debug ("seek not handled\n");

    return true;
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

    // //query segment
    // query = gst_query_new_segment (GST_FORMAT_TIME);
    // res = gst_element_query (pipeline_, query);
    // gdouble rate = -2.0;
    // gint64 start_value = -2.0;
    // gint64 stop_value = -2.0;
    // if (res) {
    //   gst_query_parse_segment (query, &rate, NULL, &start_value, &stop_value);
    //   g_debug ("rate = %f start = %" GST_TIME_FORMAT" stop = %" GST_TIME_FORMAT"\n", 
    // 	       rate,
    // 	       GST_TIME_ARGS (start_value),
    // 	       GST_TIME_ARGS (stop_value));
    // }
    // else {
    //   g_warning ("duration query failed...");
    // }
    // gst_query_unref (query);

    //query position
    query = gst_query_new_position (GST_FORMAT_TIME);
res = gst_element_query (pipeline_, query);
    gint64 cur_pos;
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

  gboolean
  Runtime::remove_quid (gpointer user_data)
  {
    QuidRemoveArgs *context = static_cast<QuidRemoveArgs *>(user_data);
    g_debug ("removing %s", context->name);
    QuiddityManager_Impl::ptr manager = context->self->manager_impl_.lock ();
    if ((bool) manager)
	manager->remove (context->name);
    else
      g_warning ("Runtime::bus_sync_handler, manager_impl cannot be locked for quiddity removing");

    delete context;
    return FALSE; //do not remove again
  }  

  GstElement * 
  Runtime::get_pipeline ()
  {
    return pipeline_;
  }

  GstBusSyncReply 
  Runtime::bus_sync_handler (GstBus * bus,
			     GstMessage * msg, gpointer user_data) 
  {
    shmdata_base_reader_t *reader = (shmdata_base_reader_t *) g_object_get_data (G_OBJECT (msg->src), 
										 "shmdata_base_reader");
    Runtime *context = static_cast<Runtime *>(user_data);
    if ( reader != NULL)
      {
	if ( shmdata_base_reader_process_error (reader, msg)) 
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
	
	gchar *quid_name = (gchar *) g_object_get_data (G_OBJECT (msg->src), "quiddity_name");
	if (quid_name != NULL)
	  {
	    QuidRemoveArgs *args = new QuidRemoveArgs ();
	    args->self = context;
	    args->name = quid_name;
	    GstUtils::g_idle_add_full_with_context (context->get_g_main_context (),
						    G_PRIORITY_DEFAULT_IDLE,
						    (GSourceFunc) remove_quid,   
						    (gpointer)args,
						    NULL);   
	  }
	//GstUtils::clean_element (GST_ELEMENT (GST_MESSAGE_SRC (msg)));
	g_error_free (error);
	return GST_BUS_DROP; 
      }
    
    return GST_BUS_PASS; 
  }

  gboolean
  Runtime::bus_called (GstBus *bus,
		       GstMessage *msg,
		       gpointer data)
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
      g_error ("bus_call Error: %s from %s", error->message, GST_MESSAGE_SRC_NAME(msg));
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

  QuiddityDocumentation 
  Runtime::get_documentation ()
  {
    return doc_;
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

