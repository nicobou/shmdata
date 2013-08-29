/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-myplugin.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_PORTMIDI_H__
#define __SWITCHER_PORTMIDI_H__

#ifndef MIDI_SYSEX
#define MIDI_SYSEX 0xf0
#endif
#ifndef MIDI_EOX
#define MIDI_EOX 0xf7
#endif

#include <map>
#include <memory>
#include <queue>
#include <portmidi.h>
#include <porttime.h>
#include <glib.h>

namespace switcher
{
  
  class PortMidi
  {
  public:
    PortMidi ();
    ~PortMidi ();

   
    //input
    int get_default_input_device_id();
    bool open_input_device(int id);
    bool close_input_device(int id);
    //bool is_queue_empty(int id);
    std::vector<unsigned char> poll(int id);
    
    //ouput
    int get_default_output_device_id();
    bool open_output_device(int id);
    bool close_output_device(int id);
    //bool send_message_to_output(int id, unsigned char status, unsigned char data1, unsigned char data2);
    
    /** Prints the list of MIDI source devices. */
    static gchar *make_devices_description (void *user_data);
    
  private:    
    class PortMidiScheduler //singleton
    {
    public:
      PortMidiScheduler();
      ~PortMidiScheduler();
      PmStream *add_input_stream(int id);
      bool remove_input_stream(PmStream *stream);
      PmEvent poll(PmStream *stream);
      bool is_queue_empty(PmStream *stream);
      
      PmStream *add_output_stream(int id);
      bool remove_output_stream(PmStream *stream);
      bool push_message(PmStream *stream, unsigned char status, unsigned char data1, unsigned char data2);
      
    private:
      std::map<PmStream *,std::queue<PmEvent> *> input_queues_;
      std::map<PmStream *,std::queue<PmEvent> *> output_queues_;
      bool portmidi_initialized_;
      bool app_sysex_in_progress_;
      bool thru_sysex_in_progress_;
      static void process_midi(PtTimestamp timestamp, void *userData);
    };
    
    gchar *devices_description_;
    static PortMidiScheduler *scheduler_;
    static guint num_of_streams_;
    std::map<guint, PmStream *> input_streams_;
    std::map<guint, PmStream *> output_streams_;
    
  };
 
}  // end of namespace

#endif // ifndef
