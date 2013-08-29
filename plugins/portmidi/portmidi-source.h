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

#ifndef __SWITCHER_PORTMIDI_SOURCE_H__
#define __SWITCHER_PORTMIDI_SOURCE_H__

#include "portmidi-devices.h"
#include "switcher/quiddity.h"
#include "switcher/custom-property-helper.h"
#include "shmdata/any-data-writer.h"

namespace switcher
{
  
  class PortMidiSource : public Quiddity, PortMidi
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PortMidiSource);
    ~PortMidiSource ();
    
  private:
    shmdata_any_writer_t *shmdata_writer_;
    
    CustomPropertyHelper::ptr custom_props_;
    gchar *devices_description_;
    GParamSpec *devices_description_spec_;
    
  };
  
  SWITCHER_DECLARE_PLUGIN(PortMidiSource);
  
}  // end of namespace

#endif // ifndef
