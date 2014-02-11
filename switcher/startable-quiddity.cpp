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

#include "startable-quiddity.h"
#include "quiddity.h"

namespace switcher
{

  StartableQuiddity::StartableQuiddity ()
  {}

  StartableQuiddity::~StartableQuiddity ()
  {}

  void
  StartableQuiddity::init_startable (void *quiddity)
  {
    Quiddity *quid = static_cast <Quiddity *> (quiddity);

    started_ = false;
    custom_props_.reset (new CustomPropertyHelper ());
    started_prop_ = 
      custom_props_->make_boolean_property ("started", 
					    "started or not",
					    (gboolean)FALSE,
					    (GParamFlags) G_PARAM_READWRITE,
					    StartableQuiddity::set_started,
					    StartableQuiddity::get_started,
					    this);
    quid->install_property_by_pspec (custom_props_->get_gobject (), 
				      started_prop_, 
				      "started",
				      "Started");
  }

  gboolean 
  StartableQuiddity::get_started (void *user_data)
  {
    StartableQuiddity *context = static_cast<StartableQuiddity *> (user_data);
    if (!context->started_)
      return FALSE;
    return TRUE;
  }

  void 
  StartableQuiddity::set_started (gboolean started, void *user_data)
  {
    StartableQuiddity *context = static_cast<StartableQuiddity *> (user_data);
    if (started)
      {
	if (context->start ())
	  context->started_ = true;
      }
    else
      {
	if (context->stop ())
	  context->started_ = false;
      }
    context->custom_props_->notify_property_changed (context->started_prop_);
  }

    bool 
    StartableQuiddity::is_started ()
    {
      return started_;
    }
  
}//end of class

