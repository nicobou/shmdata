/*
 * This file is part of switcher-pjsip.
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

#include "pjsip.h"

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PJSIP,
				       "SIP (Session Initiation Protocol)",
				       "network", 
				       "Manages user sessions",
				       "LGPL",
				       "sip",				
				       "Nicolas Bouillot");
  PJSIP::PJSIP ()
  {}

  bool
  PJSIP::init ()
  {
    init_startable (this);
    g_debug ("hello from plugin");
    return true;
  }
  
  PJSIP::~PJSIP ()
  {}
  
  bool
  PJSIP::start ()
  {
    g_debug ("start from my plugin");
    return true;
  }

  bool
  PJSIP::stop ()
  {
    g_debug ("stop from my plugin");
    return true;
  }
}
