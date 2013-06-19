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

#include "uris.h"
#include "gst-utils.h"
#include <glib/gprintf.h>
#include <memory>

namespace switcher
{
  QuiddityDocumentation Uris::doc_ ("uri decoding", "uris", 
				    "play/pause/seek/loop/synchronize multiple uris");
  
  bool
  Uris::init() 
  { 
    group_ = g_new0 (Group, 1);   

    if (!GstUtils::make_element ("bin", &group_->bin))
      return false;

    //set the name before registering properties
    set_name (gst_element_get_name (group_->bin));
    add_element_to_cleaner (group_->bin);

    group_->commands = g_async_queue_new ();
    group_->numTasks = g_async_queue_new ();
    group_->masterpad = NULL;
    group_->datastreams = g_hash_table_new (g_direct_hash, g_direct_equal);
    group_->padtoblock = g_hash_table_new (g_direct_hash, g_direct_equal);
    //allows to add and remove to/from the pipeline without disposing   
    //unref done in the group_remove function   
    gst_object_ref (group_->bin);   
    gst_bin_add (GST_BIN (bin_), group_->bin);   
    // if (!gst_element_sync_state_with_parent (group_->bin)) 
    //   g_warning ("create_group: pb sync bin state with parent"); 
    g_debug ("group created");
    group_->state = GROUP_PAUSED;    
    group_->user_data = this;

    //registering add_uri
    register_method("add_uri",
		    (void *)&add_uri_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("add_uri", 
			    "add an uri to the group (the group is looping at the end of the first uri added)", 
			    Method::make_arg_description ("uri", 
							  "the uri to add",
							  NULL));

    //registering play
    register_method("start",
		    (void *)&play_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ("start", 
			    "start the stream(s)", 
			    Method::make_arg_description ("none",
							  NULL));

    //using play pause seek from runtime
    // //registering pause
    // register_method("pause",
    // 		    (void *)&pause_wrapped, 
    // 		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
    // 		    (gpointer)this);
    // set_method_description ("pause", 
    // 			    "pause the stream(s)", 
    // 			    Method::make_arg_description ("none",
    // 							  NULL));

    // //registering seek
    // register_method("seek",
    // 		    (void *)&seek_wrapped, 
    // 		    Method::make_arg_type_description (G_TYPE_DOUBLE, NULL),
    // 		    (gpointer)this);
    // set_method_description ("seek", 
    // 			    "seek the stream(s)", 
    // 			    Method::make_arg_description ("position",
    // 							  "position in milliseconds",
    // 							  NULL));
    return true;
  }
  
  QuiddityDocumentation 
  Uris::get_documentation ()
  {
    return doc_;
  }

  gboolean
  Uris::add_uri_wrapped (gpointer uri, gpointer user_data)
  {
    Uris *context = static_cast<Uris *>(user_data);
    if (context->add_uri ((char *)uri))
      return TRUE;
    else
      return FALSE;
  }

  bool 
  Uris::add_uri (std::string uri)
  {
    g_debug ("add_uri %s", uri.c_str ());
    group_add_uri (group_, uri.c_str ());
    return true;
  }
  
  gboolean
  Uris::play_wrapped (gpointer unused, gpointer user_data)
  {
    Uris *context = static_cast<Uris *>(user_data);
      
    if (context->play ())
      return TRUE;
    else
      return FALSE;
  }
  
  bool
  Uris::play ()
  {
    g_debug ("play");
    group_play (group_);
    return true;
  }
  

  gboolean
  Uris::pause_wrapped (gpointer unused, gpointer user_data)
  {
    Uris *context = static_cast<Uris *>(user_data);
      
    if (context->pause ())
      return TRUE;
    else
      return FALSE;
  }

  bool
  Uris::pause ()
  {
    g_debug ("pause");
    group_pause (group_);
    return true;
  }
  
  gboolean
  Uris::seek_wrapped (gdouble position, gpointer user_data)
  {
    Uris *context = static_cast<Uris *>(user_data);
      
    g_debug ("seek_wrapped %f", position);

    if (context->seek (position))
      return TRUE;
    else
      return FALSE;
  }

  bool
  Uris::seek (gdouble position)
  {
    g_debug ("seek %f", position);
    group_seek (group_, position);
    return true;
  }
  
  
  //************************* wrapping c code:

  void 
  Uris::group_add_uri (Group *group, const char *uri) 
  { 
    gchar *uri_tmp;
    
    switch ( group->state ) {
    case GROUP_PAUSED:
      g_debug ("group_add_uri: setting GROUP_TO_PAUSED ");
      group->state = GROUP_TO_PAUSED;
      group_add_task (NULL, NULL, group);
      break;
    case GROUP_TO_PAUSED:
      uri_tmp = g_strdup(uri);
      group_queue_command (group, (gpointer)&group_add_uri, uri_tmp);
      return;
      break;
    case GROUP_PLAYING:
      g_debug ("group_add_uri: setting GROUP_TO_PLAYING ");
      group->state = GROUP_TO_PLAYING;
      group_add_task (NULL, NULL, group);
      break;
    case GROUP_TO_PLAYING:
      uri_tmp = g_strdup(uri);
      group_queue_command (group, (gpointer)&group_add_uri, uri_tmp);
      return;
      break;
    default:
      g_warning ("unknown state when adding uri");
      break;
    }
    
    g_debug ("adding uri");
    
    GstElement *src;
    if (!GstUtils::make_element ("uridecodebin",&src))
      return;
    
    g_signal_connect (G_OBJECT (src), 
		      "pad-added", 
		      (GCallback) uridecodebin_pad_added_cb , 
		      (gpointer) group);
    g_signal_connect (G_OBJECT (src),  
		      "no-more-pads",  
		      (GCallback) uridecodebin_no_more_pads_cb ,  
		      (gpointer) group);      
    g_signal_connect (G_OBJECT (src),  
		      "drained",  
		      (GCallback) uridecodebin_drained_cb ,  
		      (gpointer) group);      
    g_object_set (G_OBJECT (src),  
		  "ring-buffer-max-size",(guint64)200000000, 
		  "download",TRUE, 
		  "use-buffering",TRUE, 
		  "async-handling",TRUE, 
		  "buffer-duration",9223372036854775807, 
		  NULL); 
    g_object_set (G_OBJECT (src), "uri",   
		  uri, NULL);  
    
    gst_bin_add (GST_BIN (group->bin), src);
    if (!gst_element_sync_state_with_parent (src))  
      g_warning ("add_uri: pb sync uridecodebin state with parent");  
  } 
  
  
  void 
  Uris::uridecodebin_pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data)   
  {   
    Group *group = (Group *) user_data;   
    Uris *context = static_cast<Uris *>(group->user_data);
    
    //const gchar *padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));
    //detecting type of media
    const gchar *padname;
    if (0 == g_strcmp0 ("ANY", gst_caps_to_string (gst_pad_get_caps (pad))))
      padname = "custom";
    else
      padname = gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));
    
    g_debug ("padname:%s", padname);

    if (1 == 1) 
      {
	group_add_task (NULL,NULL,group);  
	Sample *sample = g_new0 (Sample,1);  
	sample->group = group;  
	sample->timeshift = 0;
	
     	GstElement *queue = gst_element_factory_make ("queue",NULL);  
     	sample->seek_element = queue;  

     	/* add to the bin */     
     	gst_bin_add_many (GST_BIN (group->bin), 
			  // continuous_stream,
			  // identity, 
			  queue,
			  NULL);     
	// gst_element_link (continuous_stream, identity);
	// gst_element_link (identity, queue);

     	GstPad *queue_srcpad = gst_element_get_static_pad (queue, "src");   
     	sample->bin_srcpad = gst_ghost_pad_new (NULL, queue_srcpad);     
     	group_do_block_datastream (sample);
     	//gst_pad_set_blocked_async (sample->bin_srcpad,TRUE,pad_blocked_cb,sample);     
     	gst_object_unref (queue_srcpad);     
     	gst_pad_set_active(sample->bin_srcpad,TRUE);     
     	gst_element_add_pad (group->bin, sample->bin_srcpad);         
     	if (group->masterpad == NULL)     
     	  {     
     	    g_debug ("masterpad is %p",sample->bin_srcpad);     
     	    group->masterpad = sample->bin_srcpad;     
     	  }     
	
     	//probing eos        
     	gst_pad_add_event_probe (sample->bin_srcpad, (GCallback) event_probe_cb, (gpointer)sample);        
     	/* gst_pad_add_buffer_probe (sample->bin_srcpad,   */
     	/* 				(GCallback) buffer_probe_cb,   */
     	/* 				(gpointer)sample);   */
	
	
     	GstPad *queue_sinkpad = gst_element_get_static_pad (queue, "sink");  
	
     	//linking newly created pad with the audioconvert_sinkpad -- FIXME should verify compatibility       
     	gst_pad_link (pad, queue_sinkpad);     
     	gst_object_unref (queue_sinkpad);  
	
     	if (!gst_element_sync_state_with_parent (queue))        
     	  g_error ("pb syncing video datastream state");      

     	//assuming object is an uridecodebin and get the uri    
     	gchar *uri;    
     	g_object_get (object,"uri",&uri,NULL);    
     	g_debug ("new sample from uri: %s",uri);    
     	//adding sample to hash table   
     	g_hash_table_insert (group->datastreams,sample,uri); //FIXME clean hash    

	//making a shmdata:
	// GstElement *identity;
	// GstUtils::make_element ("identity", &identity);
	// g_object_set (identity, 
	// 	      "sync", TRUE, 
	// 	      "single-segment", TRUE,
	// 	      NULL);
	GstElement *funnel;
	GstUtils::make_element ("funnel", &funnel);
	GstElement *continuous_stream;
	if (g_str_has_prefix (padname, "video/"))
	  {
	    GstUtils::make_element ("videomixer2", &continuous_stream);
	    //g_object_set (G_OBJECT (continuous_stream), "force-fps", 30, 1, NULL);
	  }
	else
	  GstUtils::make_element ("identity", &continuous_stream);
	GstElement *identity;
	GstUtils::make_element ("identity", &identity);
	g_object_set (identity, 
		      "sync", TRUE, 
		      "single-segment", TRUE,
		      NULL);


	gst_bin_add_many (GST_BIN (context->bin_), 
			  continuous_stream,
			  identity,
			  funnel, 
			  NULL);
	gst_element_link (funnel, continuous_stream);
	gst_element_link (continuous_stream, identity);
	GstUtils::sync_state_with_parent (identity);
	GstUtils::sync_state_with_parent (continuous_stream);
	GstUtils::sync_state_with_parent (funnel);

	//giving a name to the stream
	gchar **padname_splitted = g_strsplit_set (padname, "/",-1);
	//counting 
	int count = 0;
	if (context->media_counters_.contains (std::string (padname_splitted[0])))
	  {
	    count = context->media_counters_. lookup (std::string (padname_splitted[0]));
	    count = count+1;
	  }
	context->media_counters_.replace (std::string (padname_splitted[0]), count);
	
	gchar media_name[256];
	g_sprintf (media_name,"%s_%d",padname_splitted[0],count);
	g_debug ("uridecodebin: new media %s %d",media_name, count );
	g_strfreev(padname_splitted);
	
	//creating a shmdata
	ShmdataWriter::ptr connector;
	connector.reset (new ShmdataWriter ());
	std::string connector_name = context->make_file_name (media_name);
	connector->set_path (connector_name.c_str());
	GstCaps *caps = gst_pad_get_caps_reffed (pad);

	connector->plug (context->bin_, identity, caps);
	
	if (G_IS_OBJECT (caps))
	  gst_object_unref (caps);
	context->register_shmdata_writer (connector);
	
	g_message ("%s created a new shmdata writer (%s)", 
		   context->get_nick_name ().c_str(), 
		   connector_name.c_str ());
	
	
	//saving where to connect to the shmdata
     	sample->element_to_link_with = funnel;  
      }
    else
      {
	g_debug ("not handled data type: %s",padname);
	GstElement *fake = gst_element_factory_make ("fakesink", NULL);
	gst_bin_add (GST_BIN (group->bin),fake);
	if (!gst_element_sync_state_with_parent (fake))      
	  g_warning ("pb syncing datastream state: %s",padname);
	GstPad *fakepad = gst_element_get_static_pad (fake,"sink");
	gst_pad_link (pad,fakepad);
	gst_object_unref (fakepad);
      }
    return;   
  }   
  
  
  void 
  Uris::uridecodebin_no_more_pads_cb (GstElement* object, gpointer user_data)   
  {   
    Group *group = (Group *) user_data;   
    g_debug ("** no more pads");
    group_try_change_state (group);
  }
 
  void 
  Uris::uridecodebin_drained_cb (GstElement* object, gpointer user_data)   
  {   
    //Group *group = (Group *) user_data;   
    g_debug ("drained");
  }

  void
  Uris::group_add_task (gpointer key, gpointer value, gpointer user_data)
  {
    Group *group = (Group *) user_data;
    g_async_queue_push (group->numTasks,group);
    g_debug ("-- task added");
  }

  void
  Uris::group_queue_command (Group *group, gpointer func, gpointer arg)
  {
    GroupCommand *command = g_new0 (GroupCommand,1);
    command->func = (gboolean (*)(gpointer, gpointer))func;
    command->group = group;
    command->arg = arg;
    g_async_queue_push (group->commands,command);
    g_debug ("-- queuing command");
  }

  void 
  Uris::group_try_change_state (gpointer user_data)
  {
    Group *group = (Group *) user_data;
    
    g_debug ("try to change state %d", g_async_queue_length (group->numTasks));

    if (g_async_queue_length (group->numTasks) != 0 )
      {
	if (NULL == g_async_queue_try_pop (group->numTasks))
	  g_debug ("warning: queue not poped......................... ");
      
	if (g_async_queue_length (group->numTasks) == 0 ) 
	  { 
	    if (group->state == GROUP_TO_PLAYING) 
	      {
		group->state = GROUP_PLAYING;  
		g_debug ("back to playing state %p", &(group->state));
	      }
	    else if (group->state == GROUP_TO_PAUSED) 
	      {
		group->state = GROUP_PAUSED;
		g_message ("back to plaused state");
	      }

	    GroupCommand *command = (GroupCommand *)g_async_queue_try_pop (group->commands);
	    //launching the command with the higher priority 
	    if (command != NULL)
	      GstUtils::g_idle_add_full_with_context (((Uris *)group->user_data)->get_g_main_context (),
						      G_PRIORITY_DEFAULT,
						      &group_launch_command,
						      (gpointer)command,
						      NULL);
	    g_debug ("queue size %d",g_async_queue_length (group->numTasks));
	    g_debug ("** new command launched");
	  } 
      }  
    else 
      {
	g_debug ("nothing to do for changing the state");
      }
  }

  gboolean
  Uris::group_launch_command (gpointer user_data)
  {
    GroupCommand *command = (GroupCommand *) user_data;
    if (user_data != NULL)
      {
	(*command->func) (command->group,command->arg);
	if (command->arg != NULL)
	  {
	    g_debug ("freeing coommand arg");
	    g_free (command->arg);
	  }
	g_free (command);
      }
    return FALSE; //to be removed from the gmain loop
  }
  
  gboolean    
  Uris::group_play (Group *group)
  {   
    g_debug ("trying to play state %d, queue lenth %d",group->state, g_async_queue_length (group->numTasks));
    
    if (!GST_IS_ELEMENT(group->bin))   
      {   
	g_warning ("trying to play something that is not a GStreamer element");   
	return FALSE;   
      }   
    
    switch (group->state ) {
    case GROUP_PAUSED:
      g_debug ("*********************** play group");   
      group->state = GROUP_TO_PLAYING;   
      g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);
      g_hash_table_foreach (group->datastreams,(GHFunc)group_link_datastream,group);
      g_hash_table_foreach (group->datastreams,(GHFunc)group_unblock_datastream,group);
      break;
    case GROUP_TO_PAUSED:
      group_queue_command (group, (gpointer)&group_play_wrapped_for_commands, NULL);
      break;
    case GROUP_PLAYING:
      break;
    case GROUP_TO_PLAYING:
      break;
    default:
      g_debug ("unknown state when playing");
      break;
    }
    
    return FALSE;   
  }   

  void
  Uris::group_link_datastream (gpointer key, gpointer value, gpointer user_data)
  {
    Sample *sample = (Sample *) key;
    Group *group = (Group *) user_data;
    
    if (gst_pad_is_linked (sample->bin_srcpad))
      g_warning (".....................oups, already linked ");
    
    GstPad *peerpad;
    //trying video
    if (!GST_IS_PAD (sample->bin_srcpad)) 
      sample->bin_srcpad = gst_element_get_compatible_pad (sample->element_to_link_with,
							   sample->bin_srcpad,
							   NULL); 
    
    //FIXME everybody goes to the same mixer...
    peerpad = gst_element_get_compatible_pad (sample->element_to_link_with,
					      sample->bin_srcpad,
					      NULL);
    
    if (GST_IS_PAD (peerpad))
      {
     	g_debug ("pad_link: %d",gst_pad_link (sample->bin_srcpad, peerpad));
     	gst_object_unref (peerpad);
     	if (!gst_pad_is_linked (sample->bin_srcpad))
     	  g_debug ("oups, link did not worked");
      }
  }

  void
  Uris::group_unblock_datastream (gpointer key, gpointer value, gpointer user_data)
  {
    Sample *sample = (Sample *) key;
    Group *group = (Group *) user_data;
    
    g_debug ("group_unblock_datastream %p",sample->bin_srcpad);
 
    if (!gst_pad_is_blocked (sample->bin_srcpad) || !gst_pad_is_linked (sample->bin_srcpad))
      {
	g_warning ("OOPS - trying to unblock a not blocked pad %p",sample->bin_srcpad);
	return;
      }
    if (!gst_pad_set_blocked_async (sample->bin_srcpad,FALSE,(GstPadBlockCallback)pad_blocked_cb,sample))
      {
	g_debug  ("play_source: pb unblocking %p",sample->bin_srcpad);
      }
    group_try_change_state (sample->group);
  }

  gboolean
  Uris::group_play_wrapped_for_commands (gpointer user_data, gpointer user_data2)
  {
    Group *group = (Group *) user_data;
    return group_play (group);
  }
  
  void 
  Uris::pad_blocked_cb (GstPad *pad, gboolean blocked, gpointer user_data)
  {
    Sample *sample = (Sample *) user_data;
    if (blocked)
      {
	//unlinking and changing state only if expected
	//this is done this way because this callback is sometime called twice
	if (g_hash_table_remove (sample->group->padtoblock,sample->bin_srcpad))
	  {
	    group_unlink_datastream ((gpointer)sample,NULL,NULL);
	    g_debug ("XXX pad_blocked_cb %p",pad);
	    group_try_change_state (sample->group);
	  }
	else
	  g_debug ("pad_blocked_cb: cannot remove from hash pad to block %p",sample->bin_srcpad);
      }
    else
      {
	g_debug ("xxx pad unblocked %p",pad);
      }
  }
  
  void
  Uris::group_unlink_datastream (gpointer key, gpointer value, gpointer user_data)
  {
    Sample *sample = (Sample *) key;
    unlink_pad (sample->bin_srcpad);
  }
  
  void 
  Uris::unlink_pad (GstPad *pad)
  {
    if (!GST_IS_PAD (pad))
      {
	g_warning ("(unlink_pad): trying to unlink something not a pad");
	return;
      }
    
    GstPad *peerpad = gst_pad_get_peer (pad);
    if (GST_IS_PAD (peerpad))
      {
	GstElement *peerpadparent = gst_pad_get_parent_element (peerpad);  
	g_debug ("unlink returns %d",gst_pad_unlink (pad, peerpad));  
	// give back the pad     
	gst_element_release_request_pad (peerpadparent, peerpad);  
	gst_object_unref (peerpad);  
	gst_object_unref (peerpadparent);
      }
  }
  
  gboolean
  Uris::group_pause_wrapped_for_commands (gpointer user_data, gpointer user_data2)
  {
    Group *group = (Group *) user_data;
    return group_pause (group);
  }

  gboolean    
  Uris::group_pause (Group *group)   
  {   
    g_debug ("try to pause, state is %d",group->state);
    
    switch ( group->state ) {
    case GROUP_PAUSED:
      break;
    case GROUP_TO_PAUSED:
      //group_queue_command (group,&group_pause_wrapped_for_commands,NULL);
      break;
    case GROUP_PLAYING:
      g_debug ("*** group pause");
      group->state = GROUP_TO_PAUSED;
      g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);
      g_debug ("task added");
      g_hash_table_foreach (group->datastreams,(GHFunc)group_block_datastream_wrapped_for_hash,NULL);
      break;
    case GROUP_TO_PLAYING:
      group_queue_command (group,(gpointer)&group_pause_wrapped_for_commands,NULL);
      break;
    default:
      g_warning ("unknown state when playing");
      break;
    }
    return FALSE;
  }
  
  void
  Uris::group_block_datastream_wrapped_for_hash (gpointer key, gpointer value, gpointer user_data)
  {
    Sample *sample = (Sample *) key;
    g_debug ("group_block_datastream_wrapped_for_hash: called %p",sample->bin_srcpad);
    if (!gst_pad_is_blocked (sample->bin_srcpad))
      group_do_block_datastream (sample);
    else
      {
	g_warning ("group_block_datastream_wrapped_for_hash: WARNING not blocking unblocked pad");
	group_try_change_state (sample->group);
      }
  }
  
  void
  Uris::group_do_block_datastream (Sample *sample)
  {
    if (!gst_pad_set_blocked_async (sample->bin_srcpad,TRUE,(GstPadBlockCallback)pad_blocked_cb,sample))
      g_warning ("play_source: pb blocking");
    else
      {
	//keep the pad in memory for dequeueing task when necessary
	g_hash_table_insert (sample->group->padtoblock,sample->bin_srcpad,sample->bin_srcpad);
      }
  }
  

  void 
  Uris::group_seek_datastream (gpointer key, gpointer value, gpointer user_data)
  {
    Sample *sample = (Sample *) key;
    group_do_seek_datastream (sample);
    return;
  }
  
  gboolean
  Uris::group_seek_wrapped_for_commands (gpointer user_data, gpointer user_data2)
  {
    Group *group = (Group *) user_data;
    return group_seek (group, group->seek_position);
  }

  gboolean
  Uris::group_seek (Group *group, gdouble position)
  {
    g_debug ("trying to seek, state %d",group->state);
    
    group->seek_position = position;

    switch ( group->state ) {
    case GROUP_TO_PAUSED:
      group_queue_command (group, (gpointer)&group_seek_wrapped_for_commands, NULL);
      return FALSE;
      break;
    case GROUP_TO_PLAYING:
      group_queue_command (group, (gpointer)&group_seek_wrapped_for_commands, NULL);
      return FALSE;
      break;
    case GROUP_PAUSED:
      g_debug ("group_seek: GROUP_TO_PAUSED");
      group->state = GROUP_TO_PAUSED; //using PAUSED state for seeking
      break;
    case GROUP_PLAYING:
      g_async_queue_lock (group->commands);
      group_queue_command_unlocked (group, (gpointer)&group_pause_wrapped_for_commands, NULL);
      group_queue_command_unlocked (group, (gpointer)&group_seek_wrapped_for_commands, NULL);
      group_queue_command_unlocked (group, (gpointer)&group_play_wrapped_for_commands, NULL);
      g_async_queue_unlock (group->commands);
      group_launch_command (g_async_queue_try_pop (group->commands));
      return FALSE;
      break;
    default:
      g_warning ("unhandled state when seeking");
      break;
    }
    group_do_group_seek (group);
    return FALSE;
  }

  void
  Uris::group_do_seek_datastream (Sample *sample)
  {
    g_debug ("--------------: going to seek for a sample");
    // GstQuery *query;
    // gboolean res;
    // query = gst_query_new_segment (GST_FORMAT_TIME);
    // res = FALSE;//gst_element_query (sample->group->pipeline, query);
    // gdouble rate = -2.0;
    // gint64 start_value = -2.0;
    // gint64 stop_value = -2.0;
    // if (res) {
    //   gst_query_parse_segment (query, &rate, NULL, &start_value, &stop_value);
    //   g_debug ("rate = %f start = %"GST_TIME_FORMAT" stop = %"GST_TIME_FORMAT"", 
    // 	       rate,
    // 	       GST_TIME_ARGS (start_value),
    // 	       GST_TIME_ARGS (stop_value));
    // }
    // else {
    //   g_warning ("duration query failed...");
    // }
    // gst_query_unref (query);
    

    //FIXME get a position for seeking
    
    gboolean ret;
    ret = gst_element_seek (sample->seek_element,  
			    (gdouble)1.0,  
			    GST_FORMAT_TIME,  
			    (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE), 
			    //| GST_SEEK_FLAG_SKIP 
			    //| GST_SEEK_FLAG_KEY_UNIT, //using key unit is breaking synchronization 
			    GST_SEEK_TYPE_SET,  
			    sample->group->seek_position * GST_SECOND,  
			    GST_SEEK_TYPE_NONE,  
			    GST_CLOCK_TIME_NONE);  
    
    if (!ret)
      g_warning ("seek not handled");
    
    /* //if the seek is performed with an unlinked pad, but not blocked,  */
    /* //data will flow entirely before being linked, so blocking here  */
    /* //in that case (call this function when blocked  */
    /* if (!gst_pad_is_blocked (sample->bin_srcpad)) */
    /*   { */
    /*     g_debug ("going to seek a not blocked pad! and unlinked pad"); */
    /*   } */
    
    gst_element_get_state (sample->seek_element,  
			   NULL,  
			   NULL,  
			   GST_CLOCK_TIME_NONE);  
    g_debug ("--------------: seek done for a sample");
    group_try_change_state (sample->group);
  }

  void
  Uris::group_do_group_seek (Group *group)
  {
    g_debug ("*** group_seek");
    group_add_task (NULL,NULL,group);
    g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);
    //g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);//counting duplicated blocked signals
    g_hash_table_foreach (group->datastreams,(GHFunc)group_seek_datastream,NULL);
    group_try_change_state (group);
  }
  
  void
  Uris::group_queue_command_unlocked (Group *group, gpointer func, gpointer arg)
  {
    GroupCommand *command = g_new0 (GroupCommand,1);
    command->func = (gboolean (*)(gpointer, gpointer))func;
    command->group = group;
    command->arg = arg;
    g_async_queue_push_unlocked (group->commands,command);
    g_debug ("-- queuing (unlocked) command");
  }

  gboolean
  Uris::event_probe_cb (GstPad *pad, GstEvent * event, gpointer data)
  {
    Sample *sample = (Sample *)data;
    if (GST_EVENT_TYPE (event) == GST_EVENT_EOS) { 
      //g_debug ("EOS caught and disabled ");
      g_debug ("pad with EOS %s:%s, pointer:%p src: %p %s",
	       GST_DEBUG_PAD_NAME (pad),pad,GST_EVENT_SRC(event), gst_element_get_name (GST_EVENT_SRC(event)));
      if (sample->group->state != GROUP_PLAYING)
	{
	  g_warning ("probing EOS while not playing should not be possible %p",&(sample->group->state));
	}
      group_unlink_datastream ((gpointer)sample,NULL,NULL);   
    
      if (pad == sample->group->masterpad)  
	{  
	  // this seems not working if play rate >1.0
	  g_debug ("EOS on master pad, looping");  
	  // g_idle_add ((GSourceFunc) group_eos_rewind,   
	  // 	      (gpointer)sample->group);   
	  GstUtils::g_idle_add_full_with_context (((Uris *)sample->group->user_data)->get_g_main_context (),
						  G_PRIORITY_DEFAULT,
						  (GSourceFunc) group_eos_rewind,
						  (gpointer)sample->group,
						  NULL);
	} 
    
      return FALSE; 
    }
    g_debug ("event received :%d",GST_EVENT_TYPE (event));
    return TRUE;
  }

  gboolean
  Uris::group_eos_rewind (Group *group)
  {
    group->state = GROUP_TO_PAUSED; 
    g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);
    g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);
    g_hash_table_foreach (group->datastreams,(GHFunc)group_block_datastream_wrapped_for_hash,NULL);
    g_hash_table_foreach (group->datastreams,(GHFunc)group_seek_datastream,NULL);
    group_play (group);	     
    return FALSE;
  }
  

}
