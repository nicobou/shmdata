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


#ifndef __SWITCHER_URIS_H__
#define __SWITCHER_URIS_H__

#include "base-source.h"
#include "gst-element-cleaner.h"
#include "string-map.h"
#include <memory>

namespace switcher
{

  class Uris : public BaseSource, public GstElementCleaner
  {
  public:
    typedef std::shared_ptr<Uris> ptr;
    bool init ();
    bool add_uri (std::string uri);
    bool play ();
    bool pause ();
    bool seek (gdouble position);
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

    static gboolean add_uri_wrapped (gpointer uri, gpointer user_data);
    static gboolean play_wrapped (gpointer unused, gpointer user_data);
    static gboolean pause_wrapped (gpointer unused, gpointer user_data);
    static gboolean seek_wrapped (gdouble position, gpointer user_data);

  private: 
    StringMap<int> media_counters_;

    //wraping c code:
    typedef enum GourpState_ {
      GROUP_TO_PLAYING = 0,
      GROUP_PLAYING = 1,
      GROUP_TO_PAUSED = 2,
      GROUP_PAUSED = 3
    } GroupState;
    typedef struct
    {
      GstElement *bin/*, *audiomixer, *videomixer*/;
      GroupState state;
      GHashTable *datastreams; 
      GAsyncQueue *commands;
      //pad of the sample that determine end of play
      GstPad *masterpad;
      // a thread safe counter of task to do before actually going to a state
      // and unlocking new command
      GAsyncQueue *numTasks; 
      GHashTable *padtoblock; 
      double seek_position;
      gpointer user_data; //this
    } Group;
    typedef struct 
    {
      gboolean (*func)(gpointer, gpointer);
      Group *group;
      gpointer arg;
    } GroupCommand;
    typedef struct
    {
      GstElement *seek_element;
      // ghost pad associated with the sample
      GstPad *bin_srcpad;  
      // group to which the sample belongs
      Group *group;
      //shifting timestamps for video
      GstClockTime timeshift; 
      GstClockTime lastpause;
      GstElement *element_to_link_with;
    } Sample;
    
    Group *group_;
    static void group_add_uri (Group *group, const char *uri);
    static gboolean group_play (Group *group);
    static gboolean group_pause (Group *group);
    static gboolean group_seek (Group *group, gdouble position);
    static void uridecodebin_pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data);
    static void uridecodebin_no_more_pads_cb (GstElement* object, gpointer user_data);
    static void uridecodebin_drained_cb (GstElement* object, gpointer user_data);
    static void group_add_task (gpointer key, gpointer value, gpointer user_data);
    static void group_queue_command (Group *group, gpointer func, gpointer arg);
    static void group_try_change_state (gpointer user_data);
    static gboolean group_launch_command (gpointer user_data);
    static void group_link_datastream (gpointer key, gpointer value, gpointer user_data);
    static void group_unblock_datastream (gpointer key, gpointer value, gpointer user_data);
    static gboolean group_play_wrapped_for_commands (gpointer user_data, gpointer user_data2);
    static void pad_blocked_cb (GstPad *pad, gboolean blocked, gpointer user_data);
    static void group_unlink_datastream (gpointer key, gpointer value, gpointer user_data);
    static void unlink_pad (GstPad *pad);
    static gboolean group_pause_wrapped_for_commands (gpointer user_data, gpointer user_data2);
    static void group_block_datastream_wrapped_for_hash (gpointer key, gpointer value, gpointer user_data);
    static void group_do_block_datastream (Sample *sample);
    static void group_seek_datastream (gpointer key, gpointer value, gpointer user_data);
    static gboolean group_seek_wrapped_for_commands (gpointer user_data, gpointer user_data2);
    static void group_do_seek_datastream (Sample *sample);
    static void group_do_group_seek (Group *group);
    static void group_queue_command_unlocked (Group *group, gpointer func, gpointer arg);
    static gboolean event_probe_cb (GstPad *pad, GstEvent * event, gpointer data);
    static gboolean group_eos_rewind (Group *group);
  };

}  // end of namespace

#endif // ifndef
