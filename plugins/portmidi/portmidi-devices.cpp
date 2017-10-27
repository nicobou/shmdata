/*
 * This file is part of switcher-portmidi.
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

#include "./portmidi-devices.hpp"
#include <glib/gprintf.h>

namespace switcher {

PortMidi::PortMidiScheduler* PortMidi::scheduler_ = nullptr;
guint PortMidi::instance_counter_ = 0;

PortMidi::PortMidi() {
  if (scheduler_ == nullptr) scheduler_ = new PortMidiScheduler();
  update_device_enum();
  instance_counter_++;
}

PortMidi::~PortMidi() {
  instance_counter_--;
  if (!input_streams_.empty())
    while (input_streams_.begin() != input_streams_.end())
      close_input_device(input_streams_.begin()->first);
  if (!output_streams_.empty())
    while (output_streams_.begin() != output_streams_.end())
      close_output_device(output_streams_.begin()->first);
  if (instance_counter_ == 0) {
    delete scheduler_;
    scheduler_ = nullptr;
  }
}

BoolLog PortMidi::open_input_device(int id, on_pm_event_method method, void* user_data) {
  if (input_streams_.find(id) != input_streams_.end()) {
    return BoolLog(
        false,
        std::string("input device (id %), already opened, cannot open") + std::to_string(id));
  }

  PmStream* stream = scheduler_->add_input_stream(id, method, user_data);
  if (stream == nullptr) return BoolLog(false, "add_input_stream failled");
  input_streams_[id] = stream;
  return BoolLog(true);
}

BoolLog PortMidi::open_output_device(int id) {
  if (output_streams_.find(id) != output_streams_.end()) {
    return BoolLog(
        false,
        std::string("output device already openned, cannot open. id: ") + std::to_string(id));
  }

  PmStream* stream = scheduler_->add_output_stream(id);
  if (stream == nullptr) return false;
  output_streams_[id] = stream;
  return BoolLog(true);
}

bool PortMidi::push_midi_message(int id,
                                 unsigned char status,
                                 unsigned char data1,
                                 unsigned char data2) {
  if (output_streams_.count(id) == 0) return false;
  scheduler_->push_message(output_streams_[id], status, data1, data2);
  return true;
}

bool PortMidi::close_input_device(int id) {
  std::map<guint, PmStream*>::iterator it = input_streams_.find(id);
  if (it == input_streams_.end()) return false;
  scheduler_->remove_input_stream(it->second);
  input_streams_.erase(id);
  return true;
}

bool PortMidi::close_output_device(int id) {
  std::map<guint, PmStream*>::iterator it = output_streams_.find(id);
  if (it == output_streams_.end()) return false;

  scheduler_->remove_output_stream(it->second);
  output_streams_.erase(id);
  return true;
}

//#################################### SCHEDULER
PortMidi::PortMidiScheduler::PortMidiScheduler()
    : streams_mutex_(),
      finalize_mutex_(),
      finalizing_(FALSE),
      input_callbacks_(),
      output_queues_(),
      portmidi_initialized_(false),
      app_sysex_in_progress_(false),
      thru_sysex_in_progress_(false) {
  /* always start the timer before you start midi */
  Pt_Start(1, &process_midi, this); /* start a timer with 1 millisecond accuracy */
  /* the timer will call our function, process_midi() every millisecond */
  Pm_Initialize();
  portmidi_initialized_ = true;
}

PortMidi::PortMidiScheduler::~PortMidiScheduler() {
  portmidi_initialized_ = false;
  finalizing_ = TRUE;
  std::unique_lock<std::mutex> lock(finalize_mutex_);
  Pt_Stop(); /* stop the timer */
  Pm_Terminate();
}

PmStream* PortMidi::PortMidiScheduler::add_input_stream(int id,
                                                        on_pm_event_method method,
                                                        void* user_data) {
  PmStream* midi_in;
  if (pmNoError != Pm_OpenInput(&midi_in,
                                id,
                                nullptr /* driver info */,
                                0 /* use default input size */,
                                nullptr,
                                nullptr /* time info */))
    return nullptr;
  /* Note: if you set a filter here, then this will filter what goes
     to the MIDI THRU port. You may not want to do this.
  */
  Pm_SetFilter(midi_in, PM_FILT_ACTIVE | PM_FILT_CLOCK);

  std::unique_lock<std::mutex> lock(streams_mutex_);
  input_callbacks_[midi_in] = std::make_pair(method, user_data);
  return midi_in;
}

PmStream* PortMidi::PortMidiScheduler::add_output_stream(int id) {
  PmStream* midi_out;
  if (pmNoError != Pm_OpenOutput(&midi_out,
                                 id,
                                 nullptr /* driver info */,
                                 0 /* use default input size */,
                                 nullptr,
                                 nullptr, /* time info */
                                 0))
    return nullptr;
  std::unique_lock<std::mutex> lock(streams_mutex_);
  output_queues_[midi_out] = new std::queue<PmEvent>();
  return midi_out;
}

bool PortMidi::PortMidiScheduler::remove_input_stream(PmStream* stream) {
  {
    std::unique_lock<std::mutex> lock(streams_mutex_);
    input_callbacks_.erase(stream);
  }
  Pm_Close(stream);
  return true;
}

bool PortMidi::PortMidiScheduler::remove_output_stream(PmStream* stream) {
  {
    std::unique_lock<std::mutex> lock(streams_mutex_);
    output_queues_.erase(stream);
  }
  Pm_Close(stream);
  return true;
}

bool PortMidi::PortMidiScheduler::push_message(PmStream* stream,
                                               unsigned char status,
                                               unsigned char data1,
                                               unsigned char data2) {
  PmEvent message_to_push;
  message_to_push.message = Pm_Message(status, data1, data2);
  message_to_push.timestamp = 0;  // use current time

  output_queues_[stream]->push(message_to_push);
  return true;
}

/* timer interrupt for processing midi data.
   Incoming data is delivered to main program via in_queue.
   Outgoing data from main program is delivered via out_queue.
   Incoming data from midi_in is copied with low latency to  midi_out.
   Sysex messages from either source block messages from the other.
*/
void PortMidi::PortMidiScheduler::process_midi(PtTimestamp /*timestamp */, void* user_data) {
  PortMidiScheduler* context = static_cast<PortMidiScheduler*>(user_data);

  PmError result;
  PmEvent buffer; /* just one message at a time */
  if (context->finalizing_) return;
  if (!context->portmidi_initialized_) return;

  std::unique_lock<std::mutex> finalize_lock(context->finalize_mutex_);
  std::unique_lock<std::mutex> streams_lock(context->streams_mutex_);

  for (auto& itr : context->input_callbacks_) {
    /* see if there is any midi input to process */
    if (!context->app_sysex_in_progress_) {
      do {
        result = Pm_Poll(itr.first);
        if (result) {
          int status;
          PmError rslt = (PmError)Pm_Read(itr.first, &buffer, 1);
          if (rslt == pmBufferOverflow) continue;

          /* the data might be the end of a sysex message that
             has timed out, in which case we must ignore it.
             It's a continuation of a sysex message if status
             is actually a data byte (high-order bit is zero). */
          status = Pm_MessageStatus(buffer.message);
          if (((status & 0x80) == 0) && !context->thru_sysex_in_progress_) {
            continue; /* ignore this data */
          }

          // invoking the callback
          itr.second.first(&buffer, itr.second.second);

          /* sysex processing */
          if (status == MIDI_SYSEX)
            context->thru_sysex_in_progress_ = true;
          else if ((status & 0xF8) != 0xF8) {
            /* not MIDI_SYSEX and not real-time, so */
            context->thru_sysex_in_progress_ = false;
          }
          if (context->thru_sysex_in_progress_ && /* look for EOX */
              (((buffer.message & 0xFF) == MIDI_EOX) ||
               (((buffer.message >> 8) & 0xFF) == MIDI_EOX) ||
               (((buffer.message >> 16) & 0xFF) == MIDI_EOX) ||
               (((buffer.message >> 24) & 0xFF) == MIDI_EOX))) {
            context->thru_sysex_in_progress_ = false;
          }
        }
      } while (result);
    }
  }  // end of "for input_streams_"

  for (auto& itr : context->output_queues_) {
    /* see if there is application midi data to process */
    while (!itr.second->empty()) {
      /* see if it is time to output the next message */
      PmEvent* next = &(itr.second->front());  //(PmEvent *) Pm_QueuePeek(out_queue);
      // assert(next); /* must be non-null because queue is not empty */
      /* time to send a message, first make sure it's not blocked */
      int status = Pm_MessageStatus(next->message);
      if ((status & 0xF8) == 0xF8) {
        ; /* real-time messages are not blocked */
      } else if (context->thru_sysex_in_progress_) {
        /* maybe sysex has timed out (output becomes unblocked) */
        context->thru_sysex_in_progress_ = false;
      }

      Pm_Write(itr.first, next, 1);

      itr.second->pop();

      /* inspect message to update app_sysex_in_progress */
      if (status == MIDI_SYSEX)
        context->app_sysex_in_progress_ = true;
      else if ((status & 0xF8) != 0xF8) {
        /* not MIDI_SYSEX and not real-time, so */
        context->app_sysex_in_progress_ = false;
      }
      if (context->app_sysex_in_progress_ && /* look for EOX */
          (((buffer.message & 0xFF) == MIDI_EOX) || (((buffer.message >> 8) & 0xFF) == MIDI_EOX) ||
           (((buffer.message >> 16) & 0xFF) == MIDI_EOX) ||
           (((buffer.message >> 24) & 0xFF) == MIDI_EOX))) {
        context->app_sysex_in_progress_ = false;
      }
    }
  }
}

void PortMidi::update_device_enum() {
  int i;
  std::vector<std::string> inames;
  std::vector<std::string> inicks;
  std::vector<std::string> onames;
  std::vector<std::string> onicks;
  for (i = 0; i < Pm_CountDevices(); i++) {
    const PmDeviceInfo* listinfo = Pm_GetDeviceInfo(i);
    if (listinfo->input) {
      // warning convert nick to int instead of taking selection index
      inicks.push_back(std::to_string(i));
      inames.push_back(std::string(listinfo->name) + " (" + std::string(listinfo->interf) + ")");
    } else {
      // warning convert nick to int instead of taking selection index
      onicks.push_back(std::to_string(i));
      onames.push_back(std::string(listinfo->name) + " (" + std::string(listinfo->interf) + ")");
    }
  }
  input_devices_enum_ = Selection<>(std::make_pair(inames, inicks), 0);
  output_devices_enum_ = Selection<>(std::make_pair(onames, onicks), 0);
}

}  // namespace switcher
