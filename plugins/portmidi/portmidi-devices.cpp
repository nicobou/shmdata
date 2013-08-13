/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "portmidi-devices.h"

namespace switcher
{

  PortMidi::PortMidiScheduler *PortMidi::scheduler_ = NULL;
  uint PortMidi::num_of_streams_ = 0;


  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidi,
				       "Midi (PortMidi)",
				       "midi source sink", 
				       "midi from/to shmdata",
				       "LGPL",
				       "portmidi",				
				       "Nicolas Bouillot");

  bool
  PortMidi::init ()
  {
    if (scheduler_ == NULL)
      scheduler_ = new PortMidiScheduler();

    devices_description_ = NULL;
    make_devices_description (this);

    custom_props_.reset (new CustomPropertyHelper ());
    devices_description_spec_ = 
      custom_props_->make_string_property ("devices-json", 
					   "Description of Midi devices (json formated)",
					   devices_description_,
					   (GParamFlags) G_PARAM_READABLE,
					   NULL,
					   PortMidi::make_devices_description,
					   this);
    
    register_property_by_pspec (custom_props_->get_gobject (), 
				devices_description_spec_, 
				"devices-json",
				"Capture Devices",
				true,
				true);
    
    publish_method ("Open Input Device",
		    "open_input",
		    "open input device",
		    "success or fail",
		    Method::make_arg_description ("PortMidi Identifier",
						  "id", 
						  "PortMidi device id",
						  NULL),
  		    (Method::method_ptr) &open_input_device_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_INT, NULL),
		    true,
		    true,
		    this);

    return true;
  }
  
  PortMidi::~PortMidi ()
  {
    for (auto &it: input_streams_)
      close_input_device (it.first);
    
    for (auto &it: output_streams_)
      close_output_device (it.first);
    
    if (num_of_streams_ == 0)
      delete scheduler_;
  }
  
  int 
  PortMidi::get_default_output_device_id ()
  {
    return Pm_GetDefaultOutputDeviceID();
  }
  
  int 
  PortMidi::get_default_input_device_id ()
  {
    return Pm_GetDefaultInputDeviceID();
  }
  
  bool
  PortMidi::open_input_device (int id)
  {
    if (input_streams_.find(id) != input_streams_.end())
      {
	g_debug ("input device (id %d), already openned, cannot open", id);
	return false;
      }
    PmStream *stream = scheduler_->add_input_stream (id);
   
    if (stream == NULL)
      return false;
    num_of_streams_++;
    input_streams_[id] = stream;
    return true;
  }

  bool 
  PortMidi::open_input_device_wrapped (int id, gpointer user_data)
  {
    PortMidi *context = static_cast <PortMidi *> (user_data);
    return context->open_input_device (id);
  }

 
  bool
  PortMidi::open_output_device (int id)
  {
    if (output_streams_.find(id) != output_streams_.end())
      {
	g_debug ("output device (id %d), already openned, cannot open", id);
	return false;
      }
    PmStream *stream = scheduler_->add_output_stream (id);
    if (stream == NULL)
      return false;      

    num_of_streams_++;
    output_streams_[id] = stream;
    return true;
  }
  
  bool 
  PortMidi::open_output_device_wrapped (int id, gpointer user_data)
  {
    PortMidi *context = static_cast <PortMidi *> (user_data);
    return context->open_output_device (id);
  }
  
  bool
  PortMidi::close_input_device (int id)
  {
    std::map<uint, PmStream *>::iterator it = input_streams_.find(id);
    if (it == input_streams_.end())
      return false;
    
    if (scheduler_->remove_input_stream (it->second))
      num_of_streams_--;
    input_streams_.erase (id);
    return true;
  }

  bool 
  PortMidi::close_input_device_wrapped (int id, gpointer user_data)
  {
    PortMidi *context = static_cast <PortMidi *> (user_data);
    return context->close_input_device (id);
  }
  
  bool
  PortMidi::close_output_device (int id)
  {
    std::map<uint, PmStream *>::iterator it = output_streams_.find(id);
    if (it == output_streams_.end())
      return false;
    
    if (scheduler_->remove_output_stream (it->second))
      num_of_streams_--;
    output_streams_.erase (id);
    return true;
  }


  bool 
  PortMidi::close_output_device_wrapped (int id, gpointer user_data)
  {
    PortMidi *context = static_cast <PortMidi *> (user_data);
    return context->close_output_device (id);
  }

  
  // bool 
  //   PortMidi::is_queue_empty (int id)
  //   {
  //     if (streams_.find (id) == streams_.end())
  // 	{
  // 	  g_debug ("queue is actually not empty but the id is not managed by this instance");
  // 	  return false;
  // 	}
  //     return scheduler_->is_queue_empty (it->second);
  //   }
    
    // bool
    // PortMidi::send_message_to_output (int id, unsigned char status, unsigned char data1, unsigned char data2)
    // {
    //   if (streams_.find(id) == streams_.end())
    // 	  return false;
    //   return scheduler_->push_message (it->second, status,data1, data2);
    // }

    // // return empty vector if not accessible or <status> <data1> <data2> id success
    // std::vector<unsigned char> 
    // PortMidi::poll (int id)
    // {
    //   std::vector<unsigned char> message;
      
    //   std::map<uint, PmStream *>::iterator it = streams_.find(id);
    //   if ( it == streams_.end())
    // 	{
    // 	  g_debug ("the queue is not accessible");
    // 	  return message;
    // 	}
    //   PmEvent event = scheduler_->poll (it->second);
    //   message.push_back ((unsigned char)Pm_MessageStatus(event.message));
    //   message.push_back ((unsigned char)Pm_MessageData1(event.message));
    //   message.push_back ((unsigned char)Pm_MessageData2(event.message));
      
    //   return message;
    // } 



  gchar * 
  PortMidi::make_devices_description (void *user_data)
  {
    PortMidi *context = static_cast<PortMidi *> (user_data);
        
    if (context->devices_description_ != NULL)
      g_free (context->devices_description_);
    JSONBuilder::ptr builder (new JSONBuilder ());
    builder->reset();
    builder->begin_object ();
    builder->set_member_name ("devices");
    builder->begin_array ();

      /* list device information */
      int i;
      for (i = 0; i < Pm_CountDevices(); i++) {
	const PmDeviceInfo *listinfo = Pm_GetDeviceInfo(i);
	builder->begin_object ();
	builder->add_string_member ("long name", listinfo->name);
	builder->add_string_member ("interface", listinfo->interf);
	gchar *id = g_strdup_printf ("%d",i);
	builder->add_string_member ("id", id);
	g_free (id);
	if (listinfo->input) 
	  builder->add_string_member ("type", "input");
	else 
	  builder->add_string_member ("type","output");
	builder->end_object ();
      }

    builder->end_array ();
    builder->end_object ();
    context->devices_description_ = g_strdup (builder->get_string (true).c_str ());
    return context->devices_description_;
  }



  //#################################### SCHEDULER
    PortMidi::PortMidiScheduler::PortMidiScheduler() :  
      process_midi_exit_flag_ (false),
      app_sysex_in_progress_ (false),
      thru_sysex_in_progress_ (false)
    {
      
      portmidi_initialized_ = false;
      /* always start the timer before you start midi */
      Pt_Start(1, &process_midi, this); /* start a timer with millisecond accuracy */
      /* the timer will call our function, process_midi() every millisecond */
      Pm_Initialize();
      portmidi_initialized_ = true;

    }

    
    PortMidi::PortMidiScheduler::~PortMidiScheduler()
    {
      
      /* the timer thread could be in the middle of accessing PortMidi stuff */
      /* to detect that it is done, we first clear process_midi_exit_flag and
	 then wait for the timer thread to set it
      */
      process_midi_exit_flag_ = false;
      portmidi_initialized_ = false;
      /* busy wait for flag from timer thread that it is done */
      while (!process_midi_exit_flag_) ;
      /* at this point, midi thread is inactive and we need to shut down
       * the midi input and output
       */
      Pt_Stop(); /* stop the timer */
      Pm_Terminate();

    }



    PmStream *
    PortMidi::PortMidiScheduler::add_input_stream(int id)
    {

      PmStream *midi_in;
      if (pmNoError != Pm_OpenInput(&midi_in, 
				    id, 
				    NULL /* driver info */,
				    0 /* use default input size */,
				    NULL,
				    NULL /* time info */))
	return NULL;
      /* Note: if you set a filter here, then this will filter what goes
	 to the MIDI THRU port. You may not want to do this.
      */
      Pm_SetFilter(midi_in, PM_FILT_ACTIVE | PM_FILT_CLOCK);

      input_queues_[midi_in] = new std::queue<PmEvent>();      
     
      return midi_in;
    }

    PmStream *
    PortMidi::PortMidiScheduler::add_output_stream(int id)
    {
      
      PmStream *midi_out;
      if (pmNoError != Pm_OpenOutput(&midi_out, 
				     id, 
				     NULL /* driver info */,
				     0 /* use default input size */,
				     NULL,
				     NULL, /* time info */
				     0))
	return NULL;

      output_queues_[midi_out] = new std::queue<PmEvent>();      

      return midi_out;
    }

    PmEvent 
    PortMidi::PortMidiScheduler::poll(PmStream *stream)
    {
      PmEvent message = input_queues_[stream]->front();
      input_queues_[stream]->pop();
      return message;
    }

    bool 
    PortMidi::PortMidiScheduler::is_queue_empty(PmStream *stream)
    {
      return input_queues_[stream]->empty();
    }

    bool
    PortMidi::PortMidiScheduler::remove_input_stream(PmStream *stream)
    {
      input_queues_.erase(stream);
      return true;
    }

    bool
    PortMidi::PortMidiScheduler::remove_output_stream(PmStream *stream)
    {
      output_queues_.erase(stream);
      return true;
    }

    bool 
    PortMidi::PortMidiScheduler::push_message (PmStream *stream, 
					       unsigned char status, 
					       unsigned char data1, 
					       unsigned char data2)
    {
      PmEvent message_to_push;
      message_to_push.message = Pm_Message(status,data1,data2);
      message_to_push.timestamp=0; //use current time
      
      output_queues_[stream]->push (message_to_push);
      return true;
    }

    /* timer interrupt for processing midi data.
       Incoming data is delivered to main program via in_queue.
       Outgoing data from main program is delivered via out_queue.
       Incoming data from midi_in is copied with low latency to  midi_out.
       Sysex messages from either source block messages from the other.
    */
    void 
    PortMidi::PortMidiScheduler::process_midi(PtTimestamp timestamp, void *user_data)
    {

      PortMidiScheduler *context = static_cast<PortMidiScheduler *>(user_data);
    
      PmError result;
      PmEvent buffer; /* just one message at a time */


      /* do nothing until initialization completes */
      if (!context->portmidi_initialized_) {
	/* this flag signals that no more midi processing will be done */
	context->process_midi_exit_flag_ = true;
	return;
      }
      
      std::map<PmStream *,std::queue<PmEvent> *>::iterator itr;
      
      for(itr = context->input_queues_.begin(); itr != context->input_queues_.end(); ++itr)
	{
	  /* see if there is any midi input to process */
	  if (!context->app_sysex_in_progress_) {
	    do {
	      result = Pm_Poll(itr->first);
	      if (result) {
		int status;
		PmError rslt = (PmError)Pm_Read(itr->first, &buffer, 1);
		if (rslt == pmBufferOverflow) 
		  continue;
	
		/* the data might be the end of a sysex message that
		   has timed out, in which case we must ignore it.
		   It's a continuation of a sysex message if status
		   is actually a data byte (high-order bit is zero). */
		status = Pm_MessageStatus(buffer.message);
		if (((status & 0x80) == 0) && !context->thru_sysex_in_progress_) {
		  continue; /* ignore this data */
		}
		
		g_print ("midi input msg: %u %u %u\n",
			 Pm_MessageStatus(buffer.message),
			 Pm_MessageData1(buffer.message),
			 Pm_MessageData2(buffer.message));
		  
		itr->second->push(buffer);

		/* sysex processing */
		if (status == MIDI_SYSEX) context->thru_sysex_in_progress_ = true;
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
	}//end of "for input_streams_"


    for(itr = context->output_queues_.begin(); itr != context->output_queues_.end(); ++itr)
	{
	  /* see if there is application midi data to process */
	  while (!itr->second->empty()) {
	    /* see if it is time to output the next message */
	    PmEvent *next = &(itr->second->front());//(PmEvent *) Pm_QueuePeek(out_queue);
	    //assert(next); /* must be non-null because queue is not empty */
	    /* time to send a message, first make sure it's not blocked */
	    int status = Pm_MessageStatus(next->message);
	    if ((status & 0xF8) == 0xF8) {
	      ; /* real-time messages are not blocked */
	    } else if (context->thru_sysex_in_progress_) {
                /* maybe sysex has timed out (output becomes unblocked) */
            	  context->thru_sysex_in_progress_ = false;
	    }
	    
	    g_print ("midi input msg: %u %u %u\n",
		     Pm_MessageStatus(next->message),
		     Pm_MessageData1(next->message),
		     Pm_MessageData2(next->message));
	    
	    Pm_Write(itr->first, next, 1);
	    
	    itr->second->pop();
	    
	    /* inspect message to update app_sysex_in_progress */
	    if (status == MIDI_SYSEX) context->app_sysex_in_progress_ = true;
	    else if ((status & 0xF8) != 0xF8) {
	      /* not MIDI_SYSEX and not real-time, so */
	      context->app_sysex_in_progress_ = false;
	    }
	    if (context->app_sysex_in_progress_ && /* look for EOX */
		(((buffer.message & 0xFF) == MIDI_EOX) ||
		 (((buffer.message >> 8) & 0xFF) == MIDI_EOX) ||
		 (((buffer.message >> 16) & 0xFF) == MIDI_EOX) ||
		 (((buffer.message >> 24) & 0xFF) == MIDI_EOX))) {
	      context->app_sysex_in_progress_ = false;
	    }
	    
	  }
	}
    
    }

}
