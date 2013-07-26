/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "gtkvideo.h"
#include "switcher/gst-utils.h"
#include "switcher/quiddity-command.h"
#include <gst/gst.h>
#include <gtk/gtk.h>
#include <gst/interfaces/xoverlay.h>
#include <gdk/gdk.h>
#include <gdk/gdkx.h>


namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GTKVideo, 
				       "Video Display",
				       "video sink", 
				       "Video window with fullscreen",
				       "GPL",
				       "gtkvideosink",
				       "Nicolas Bouillot");
  
  bool
  GTKVideo::init ()
  {
    if (!GstUtils::make_element ("xvimagesink", &xvimagesink_))
      return false;

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
  
  GTKVideo::GTKVideo ()
  {
  }

  GTKVideo::~GTKVideo ()
  {
    GstUtils::clean_element (xvimagesink_);
    if (on_error_command_ != NULL)
      delete on_error_command_;
    
  }
  


}
