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

#include "switcher/uris.h"
#include "switcher/gst-utils.h"
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
    //   g_warning ("create_group: pb sync bin state with parent\n"); 
    g_debug ("group created\n");
    group_->state = GROUP_PAUSED;    

    //registering add_uri
    register_method("add_uri",
		    (void *)&add_uri_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("add_uri", 
			    "add an uri to the group", 
			    Method::make_arg_description ("uri", 
							  "the uri to add",
							  NULL));

    //registering play
    register_method("play",
		    (void *)&play_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ("play", 
			    "play the stream(s)", 
			    Method::make_arg_description ("none",
							  NULL));

    //registering pause
    register_method("pause",
		    (void *)&pause_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ("pause", 
			    "pause the stream(s)", 
			    Method::make_arg_description ("none",
							  NULL));

    //registering seek
    register_method("seek",
		    (void *)&seek_wrapped, 
		    Method::make_arg_type_description (G_TYPE_NONE, NULL),
		    (gpointer)this);
    set_method_description ("seek", 
			    "seek the stream(s)", 
			    Method::make_arg_description ("none",
							  NULL));
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
  Uris::play_wrapped (gpointer user_data)
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
  Uris::pause_wrapped (gpointer user_data)
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
  Uris::seek_wrapped (gpointer user_data)
  {
    Uris *context = static_cast<Uris *>(user_data);
      
    if (context->seek ())
      return TRUE;
    else
      return FALSE;
  }

  bool
  Uris::seek ()
  {
    g_debug ("seek");
    group_seek (group_);
    return true;
  }
  
  
  //************************* wrapping c code:

  void 
  Uris::group_add_uri (Group *group, const char *uri) 
  { 
    gchar *uri_tmp;
    
    switch ( group->state ) {
    case GROUP_PAUSED:
      g_debug ("group_add_uri: setting GROUP_TO_PAUSED\n ");
      group->state = GROUP_TO_PAUSED;
      group_add_task (NULL, NULL, group);
      break;
    case GROUP_TO_PAUSED:
      uri_tmp = g_strdup(uri);
      group_queue_command (group, (gpointer)&group_add_uri, uri_tmp);
      return;
      break;
    case GROUP_PLAYING:
      g_debug ("group_add_uri: setting GROUP_TO_PLAYING\n ");
      group->state = GROUP_TO_PLAYING;
      group_add_task (NULL, NULL, group);
      break;
    case GROUP_TO_PLAYING:
      uri_tmp = g_strdup(uri);
      group_queue_command (group, (gpointer)&group_add_uri, uri_tmp);
      return;
      break;
    default:
      g_warning ("unknown state when adding uri\n");
      break;
    }
    
    g_debug ("*********************** adding uri\n");
    
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
      g_warning ("add_uri: pb sync uridecodebin state with parent\n");  
  } 
  
  
  void 
  Uris::uridecodebin_pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data)   
  {   
    Group *group = (Group *) user_data;   
    
    const gchar *padname= gst_structure_get_name (gst_caps_get_structure(gst_pad_get_caps (pad),0));
  
    // if (g_str_has_prefix (padname, "audio/"))
    //   {
    //   //adding a task to wait for launching next command
    // 	group_add_task (NULL,NULL,group);
	
    // 	Sample *sample = g_new0 (Sample,1);
    // 	sample->group = group;
	
    // 	GstElement *audioconvert = gst_element_factory_make ("audioconvert",NULL);   
    // 	GstElement *pitch = gst_element_factory_make ("pitch",NULL);   
    // 	GstElement *resample = gst_element_factory_make ("audioresample",NULL);   
    // 	sample->seek_element = resample;
    // 	g_object_set (G_OBJECT (resample), "quality", 10 , NULL);  
    // 	g_object_set (G_OBJECT (resample), "filter-length", 2048 , NULL); 
	
    // 	/* add to the bin */   
    // 	gst_bin_add (GST_BIN (group->bin), audioconvert);   
    // 	gst_bin_add (GST_BIN (group->bin), pitch);   
    // 	gst_bin_add (GST_BIN (group->bin), resample);   
	
    // 	if (!gst_element_sync_state_with_parent (audioconvert))      
    // 	  g_error ("pb syncing audio datastream state\n");      
    // 	if (!gst_element_sync_state_with_parent (pitch))      
    // 	  g_error ("pb syncing audio datastream state\n");      
    // 	if (!gst_element_sync_state_with_parent (resample))      
    // 	  g_error ("pb syncing audio datastream state\n");      
	
    // 	GstPad *resample_srcpad = gst_element_get_static_pad (resample, "src");
    // 	sample->bin_srcpad = gst_ghost_pad_new (NULL, resample_srcpad);
    // 	group_do_block_datastream (sample);
    // 	//gst_pad_set_blocked_async (sample->bin_srcpad,TRUE,pad_blocked_cb,sample);
    // 	gst_object_unref (resample_srcpad);
    // 	gst_pad_set_active(sample->bin_srcpad,TRUE);
    // 	gst_element_add_pad (group->bin, sample->bin_srcpad);    
    // 	if (group->masterpad == NULL)
    // 	  {
    // 	    g_print ("---------------- masterpad is %p\n",sample->bin_srcpad);
    // 	    group->masterpad = sample->bin_srcpad;
    // 	  }
    // 	//probing eos   
    // 	gst_pad_add_event_probe (sample->bin_srcpad, (GCallback) event_probe_cb, (gpointer)sample);   
	
    // 	GstPad *audioconvert_sinkpad = gst_element_get_static_pad (audioconvert, "sink");
	
    // 	//linking newly create pad with the audioconvert_sinkpad -- FIXME should verify compatibility     
    // 	gst_pad_link (pad, audioconvert_sinkpad);   
    // 	gst_object_unref (audioconvert_sinkpad);
    // 	gst_element_link_many (audioconvert, pitch, resample,NULL);   
	
    // 	//g_print ("%s\n",gst_object_get_name(GST_OBJECT_CAST(object)));
	
    // 	//assuming object is an uridecodebin and get the uri 
    // 	gchar *uri; 
    // 	g_object_get (object,"uri",&uri,NULL); 
    // 	g_print ("new sample -- uri: %s\n",uri); 
    // 	//adding sample to hash table
    // 	g_hash_table_insert (group->datastreams,sample,uri); //FIXME clean hash 
	
    //   }
    // else if (g_str_has_prefix (padname, "video/"))
    //   {
	
    // 	group_add_task (NULL,NULL,group);  
    // 	Sample *sample = g_new0 (Sample,1);  
    // 	sample->group = group;  
    // 	sample->timeshift = 0;
	
    // 	GstElement *queue = gst_element_factory_make ("queue",NULL);  
    // 	sample->seek_element = queue;  
	
    // 	/* add to the bin */     
    // 	gst_bin_add (GST_BIN (group->bin), queue);     
	
    // 	GstPad *queue_srcpad = gst_element_get_static_pad (queue, "src");   
	
    // 	sample->bin_srcpad = gst_ghost_pad_new (NULL, queue_srcpad);     
    // 	group_do_block_datastream (sample);
    // 	//gst_pad_set_blocked_async (sample->bin_srcpad,TRUE,pad_blocked_cb,sample);     
    // 	gst_object_unref (queue_srcpad);     
    // 	gst_pad_set_active(sample->bin_srcpad,TRUE);     
    // 	gst_element_add_pad (group->bin, sample->bin_srcpad);         
    // 	if (group->masterpad == NULL)     
    // 	  {     
    // 	    g_print ("---------------- masterpad is %p\n",sample->bin_srcpad);     
    // 	    group->masterpad = sample->bin_srcpad;     
    // 	  }     
	
    // 	//probing eos        
    // 	gst_pad_add_event_probe (sample->bin_srcpad, (GCallback) event_probe_cb, (gpointer)sample);        
    // 	/* gst_pad_add_buffer_probe (sample->bin_srcpad,   */
    // 	/* 				(GCallback) buffer_probe_cb,   */
    // 	/* 				(gpointer)sample);   */
	
	
    // 	GstPad *queue_sinkpad = gst_element_get_static_pad (queue, "sink");  
	
    // 	//linking newly created pad with the audioconvert_sinkpad -- FIXME should verify compatibility       
    // 	gst_pad_link (pad, queue_sinkpad);     
    // 	gst_object_unref (queue_sinkpad);  
	
    // 	if (!gst_element_sync_state_with_parent (queue))        
    // 	  g_error ("pb syncing video datastream state\n");      
	
    // 	//assuming object is an uridecodebin and get the uri    
    // 	gchar *uri;    
    // 	g_object_get (object,"uri",&uri,NULL);    
    // 	g_print ("new sample -- uri: %s\n",uri);    
    // 	//adding sample to hash table   
    // 	g_hash_table_insert (group->datastreams,sample,uri); //FIXME clean hash    
	
    // 	group->videomixer = inputselector;  
	
    //   }
    // else
    {
      g_print ("not handled data type: %s\n",padname);
      GstElement *fake = gst_element_factory_make ("fakesink", NULL);
      gst_bin_add (GST_BIN (group->bin),fake);
      if (!gst_element_sync_state_with_parent (fake))      
	g_warning ("pb syncing datastream state: %s\n",padname);
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
	  g_debug ("warning: queue not poped.........................\n ");
      
	if (g_async_queue_length (group->numTasks) == 0 ) 
	  { 
	    if (group->state == GROUP_TO_PLAYING) 
	      {
		group->state = GROUP_PLAYING;  
		g_debug ("back to playing state %p\n", &(group->state));
	      }
	    else if (group->state == GROUP_TO_PAUSED) 
	      {
		group->state = GROUP_PAUSED;
		g_print ("back to plaused state\n");
	      }

	    GroupCommand *command = (GroupCommand *)g_async_queue_try_pop (group->commands);
	    //launching the command with the higher priority 
	    if (command != NULL)
	      g_idle_add_full (G_PRIORITY_DEFAULT,
			       &group_launch_command,
			       (gpointer)command,
			       NULL);
	    g_debug ("queue size %d",g_async_queue_length (group->numTasks));
	    g_debug ("** new command launched");
	  } 
      }  
    else 
      {
	g_debug ("nothing to do for changing the state\n");
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
    

    //FIXME link to something
    // if (gst_pad_is_linked (sample->bin_srcpad))
    // 	g_warning (".....................oups, already linked \n");

    // GstPad *peerpad;
    // peerpad = gst_element_get_compatible_pad (group->audiomixer,sample->bin_srcpad,NULL);
    // if (!GST_IS_PAD (peerpad))
    //   {
    // 	//trying video
    // 	if (!GST_IS_PAD (sample->bin_srcpad)) 
    // 	    sample->bin_srcpad = gst_element_get_compatible_pad (group->videomixer,sample->bin_srcpad,NULL); 
    // 	peerpad = gst_element_get_compatible_pad (group->videomixer,sample->bin_srcpad,NULL);
    // }
    
    // if (GST_IS_PAD (peerpad))
    //   {
    // 	g_debug ("pad_link: %d \n",gst_pad_link (sample->bin_srcpad, peerpad));
    // 	gst_object_unref (peerpad);
    // 	if (!gst_pad_is_linked (sample->bin_srcpad))
    // 	  g_print (".....................oups, link did not worked \n");
    //   }
  }

  void
  Uris::group_unblock_datastream (gpointer key, gpointer value, gpointer user_data)
  {
    Sample *sample = (Sample *) key;
    Group *group = (Group *) user_data;
    
    g_debug ("group_unblock_datastream %p\n",sample->bin_srcpad);
 
    if (!gst_pad_is_blocked (sample->bin_srcpad) || !gst_pad_is_linked (sample->bin_srcpad))
      {
	g_warning ("OOPS - trying to unblock a not blocked pad %p\n",sample->bin_srcpad);
	return;
      }
    if (!gst_pad_set_blocked_async (sample->bin_srcpad,FALSE,(GstPadBlockCallback)pad_blocked_cb,sample))
      {
	g_debug  ("play_source: pb unblocking %p\n",sample->bin_srcpad);
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
	    //g_print ("pad blocked and unlinked\n");
	    g_debug ("XXX pad_blocked_cb %p\n",pad);
	    group_try_change_state (sample->group);
	  }
	else
	  g_debug ("pad_blocked_cb: cannot remove from hash pad to block %p\n",sample->bin_srcpad);
      }
    else
      g_debug ("xxx pad unblocked %p\n",pad);
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
	g_warning ("(unlink_pad): trying to unlink something not a pad\n");
	return;
      }
    
    GstPad *peerpad = gst_pad_get_peer (pad);
    if (GST_IS_PAD (peerpad))
      {
	GstElement *peerpadparent = gst_pad_get_parent_element (peerpad);  
	g_debug ("unlink returns %d\n",gst_pad_unlink (pad, peerpad));  
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
    g_debug ("try to pause, state is %d\n",group->state);
    
    switch ( group->state ) {
    case GROUP_PAUSED:
      break;
    case GROUP_TO_PAUSED:
      //group_queue_command (group,&group_pause_wrapped_for_commands,NULL);
      break;
    case GROUP_PLAYING:
      g_debug ("*** group pause\n");
      group->state = GROUP_TO_PAUSED;
      g_hash_table_foreach (group->datastreams,(GHFunc)group_add_task,group);
      g_debug ("task added\n");
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
    g_debug ("group_block_datastream_wrapped_for_hash: called %p\n",sample->bin_srcpad);
    if (!gst_pad_is_blocked (sample->bin_srcpad))
      group_do_block_datastream (sample);
    else
      {
	g_warning ("group_block_datastream_wrapped_for_hash: WARNING not blocking unblocked pad\n");
	group_try_change_state (sample->group);
      }
  }
  
  void
  Uris::group_do_block_datastream (Sample *sample)
  {
    if (!gst_pad_set_blocked_async (sample->bin_srcpad,TRUE,(GstPadBlockCallback)pad_blocked_cb,sample))
      g_print ("play_source: pb blocking\n");
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
    return group_seek (group);
  }

  gboolean
  Uris::group_seek (Group *group)
  {
    g_debug ("trying to seek, state %d\n",group->state);
    
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
      g_debug ("group_seek: GROUP_TO_PAUSED\n");
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
      g_warning ("unhandled state when seeking\n");
      break;
    }
    group_do_group_seek (group);
    return FALSE;
  }

  void
  Uris::group_do_seek_datastream (Sample *sample)
  {
    g_debug ("--------------: going to seek for a sample\n");
    // GstQuery *query;
    // gboolean res;
    // query = gst_query_new_segment (GST_FORMAT_TIME);
    // res = FALSE;//gst_element_query (sample->group->pipeline, query);
    // gdouble rate = -2.0;
    // gint64 start_value = -2.0;
    // gint64 stop_value = -2.0;
    // if (res) {
    //   gst_query_parse_segment (query, &rate, NULL, &start_value, &stop_value);
    //   g_print ("rate = %f start = %"GST_TIME_FORMAT" stop = %"GST_TIME_FORMAT"\n", 
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
    ret = TRUE;// gst_element_seek (sample->seek_element,  
    // 		    1.0,  
    // 		    GST_FORMAT_TIME,  
    // 		    GST_SEEK_FLAG_FLUSH 
    // 		    | GST_SEEK_FLAG_ACCURATE, 
    // 		    //| GST_SEEK_FLAG_SKIP 
    // 		    //| GST_SEEK_FLAG_KEY_UNIT, //using key unit is breaking synchronization 
    // 		    GST_SEEK_TYPE_SET,  
    // 		    310.0 * GST_SECOND,  
    // 		    GST_SEEK_TYPE_NONE,  
    // 		    GST_CLOCK_TIME_NONE);  
    
    if (!ret)
      g_warning ("seek not handled\n");
    
    /* //if the seek is performed with an unlinked pad, but not blocked,  */
    /* //data will flow entirely before being linked, so blocking here  */
    /* //in that case (call this function when blocked  */
    /* if (!gst_pad_is_blocked (sample->bin_srcpad)) */
    /*   { */
    /*     g_print ("going to seek a not blocked pad! and unlinked pad\n"); */
    /*   } */
    
    gst_element_get_state (sample->seek_element,  
			   NULL,  
			   NULL,  
			   GST_CLOCK_TIME_NONE);  
    g_debug ("--------------: seek done for a sample\n");
    group_try_change_state (sample->group);
  }

  void
  Uris::group_do_group_seek (Group *group)
  {
    g_debug ("*** group_seek\n");
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
    g_debug ("-- queuing (unlocked) command\n");
  }
}
