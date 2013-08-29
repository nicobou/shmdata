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

#include "portmidi-source.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PortMidiSource,
				       "Midi (PortMidi)",
				       "midi source", 
				       "midi to shmdata and properties",
				       "LGPL",
				       "midisrc",
				       "Nicolas Bouillot");

  bool
  PortMidiSource::init ()
  {
    // shmdata_writer_ = shmdata_any_writer_init ();
    
    // if (! shmdata_any_writer_set_path (shmdata_writer_, "/tmp/midi_truc"))
    // {
    //   g_debug ("**** The file exists, therefore a shmdata cannot be operated with this path.\n");
    //   shmdata_any_writer_close (shmdata_writer_);
    //   return false;
    // }
    // shmdata_any_writer_set_debug (shmdata_writer_, SHMDATA_ENABLE_DEBUG);
    // shmdata_any_writer_set_data_type (shmdata_writer_, "audio/midi");
    // shmdata_any_writer_start (shmdata_writer_);

    return true;
  }
  
  PortMidiSource::~PortMidiSource ()
  {
    //shmdata_any_writer_close (shmdata_writer_);
  }
  
}
