/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include "decodebin2.h"
#include "gst-element-cleaner.h"
#include "gst-utils.h"
#include <glib/gprintf.h>
#include "scope-exit.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Decodebin2,
				       "Shmdata Decoder",
				       "decoder", 
				       "connect to a shmdata, decode it and write decoded frames to shmdata(s)",
				       "LGPL",
				       "decodebin", 
				       "Nicolas Bouillot");
  
  Decodebin2::Decodebin2 () :
    decodebin_ (new DecodebinToShmdata (*(static_cast <GPipe *> (this)))),
    media_counters_ ()
  {}

  bool
  Decodebin2::init_gpipe () 
  { 
    std::string str (__FUNCTION__); g_print ("begin %s\n", str.c_str ()); On_scope_exit {g_print ("end %s\n", str.c_str ());};
    decodebin_->invoke (std::bind(&SinglePadGstSink::set_sink_element,
				  this,
				  std::placeholders::_1));
    SinglePadGstSink::set_on_first_data_hook (Decodebin2::make_decodebin_active, this);
    return true;
  }

  void
  Decodebin2::make_decodebin_active (ShmdataReader *caller, void *user_data)
  {
    std::string str (__FUNCTION__); g_print ("begin %s\n", str.c_str ()); On_scope_exit {g_print ("end %s\n", str.c_str ());};
    Decodebin2 *context = static_cast<Decodebin2 *>(user_data);
    // context->decodebin_->invoke (std::bind(&SinglePadGstSink::set_sink_element,
    //  					   context,
    //  					   std::placeholders::_1));
    context->decodebin_->invoke (std::bind (gst_bin_add,
    					   GST_BIN (context->bin_),
    					   std::placeholders::_1));
    context->decodebin_->invoke (std::bind (GstUtils::sync_state_with_parent,
    					   std::placeholders::_1));
  }
   
}
