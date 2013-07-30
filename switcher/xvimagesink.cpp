/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "xvimagesink.h"
#include "gst-utils.h"
#include "quiddity-command.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Xvimagesink, 
				       "Video Display",
				       "video sink", 
				       "Video window with minimal features",
				       "GPL",
				       "videosink",
				       "Nicolas Bouillot");
  
  bool
  Xvimagesink::init ()
  {
#if HAVE_OSX
    if (!GstUtils::make_element ("osxvideosink", &xvimagesink_))
      return false;
 #else
    if (!GstUtils::make_element ("xvimagesink", &xvimagesink_))
      return false;
#endif

    g_object_set (G_OBJECT (xvimagesink_),
		  "force-aspect-ratio", TRUE,
		  "draw-borders", TRUE,
		  NULL);
    register_property (G_OBJECT (xvimagesink_),
		       "force-aspect-ratio",
		       "force-aspect-ratio", 
		       "Force Aspect Ratio");
    
    //set the name before registering properties
    set_name (gst_element_get_name (xvimagesink_));
    g_object_set (G_OBJECT (xvimagesink_), "sync", FALSE, NULL);

    on_error_command_ = new QuiddityCommand ();
    on_error_command_->id_ = QuiddityCommand::remove;
    on_error_command_->add_arg (get_nick_name ());

    g_object_set_data (G_OBJECT (xvimagesink_), 
     		       "on-error-command",
     		       (gpointer)on_error_command_);
    
    set_sink_element (xvimagesink_);

    return true;
  }
  
  Xvimagesink::Xvimagesink ()
  {
  }

  Xvimagesink::~Xvimagesink ()
  {
    GstUtils::clean_element (xvimagesink_);
    if (on_error_command_ != NULL)
      delete on_error_command_;
    
  }
  
}
