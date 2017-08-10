/*
 * This file is part of switcher-portmidi.
 *
 * switcher-portmidi is free software; you can redistribute it and/or
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

#include <portmidi.h>
#include <porttime.h>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include "switcher/bool-log.hpp"
#include "switcher/selection.hpp"

namespace switcher {
class PortMidi {
 public:
  typedef void (*on_pm_event_method)(PmEvent* midi_event, void* user_data);
  PortMidi();
  virtual ~PortMidi();
  PortMidi(const PortMidi&) = delete;
  PortMidi& operator=(const PortMidi&) = delete;

 protected:
  // info
  Selection<> input_devices_enum_{{"none"}, 0};
  Selection<> output_devices_enum_{{"none"}, 0};
  // input
  // static int get_default_input_device_id();
  BoolLog open_input_device(int id, on_pm_event_method method, void* user_data);
  bool close_input_device(int id);
  // bool is_queue_empty(int id);
  std::vector<unsigned char> poll(int id);
  // ouput
  // static int get_default_output_device_id();
  BoolLog open_output_device(int id);
  bool close_output_device(int id);
  bool push_midi_message(int id, unsigned char status, unsigned char data1, unsigned char data2);

 private:
  std::map<guint, PmStream*> input_streams_{};
  std::map<guint, PmStream*> output_streams_{};
  /** Prints the list of MIDI source devices. */
  void update_device_enum();

  // internal midi scheduler
  class PortMidiScheduler  // singleton
  {
   public:
    PortMidiScheduler();
    ~PortMidiScheduler();
    PmStream* add_input_stream(int id, on_pm_event_method method, void* user_data);
    bool remove_input_stream(PmStream* stream);

    PmStream* add_output_stream(int id);
    bool remove_output_stream(PmStream* stream);
    bool push_message(PmStream* stream,
                      unsigned char status,
                      unsigned char data1,
                      unsigned char data2);

   private:
    std::mutex streams_mutex_;
    std::mutex finalize_mutex_;
    gboolean finalizing_;
    std::map<PmStream*, std::pair<on_pm_event_method, void*>> input_callbacks_;
    std::map<PmStream*, std::queue<PmEvent>*> output_queues_;
    bool portmidi_initialized_;
    bool app_sysex_in_progress_;
    bool thru_sysex_in_progress_;
    static void process_midi(PtTimestamp timestamp, void* userData);
  };  // end of PortMidiScheduler
  static PortMidiScheduler* scheduler_;
  static guint instance_counter_;
};

}  // namespace switcher
#endif
