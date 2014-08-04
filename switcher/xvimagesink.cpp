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
				       "LGPL",
				       "videosink",
				       "Nicolas Bouillot");
  

  Xvimagesink::Xvimagesink () :
    sink_bin_ (nullptr),
    queue_ (nullptr),
    ffmpegcolorspace_ (nullptr),
    xvimagesink_ (nullptr)
  {}

  Xvimagesink::~Xvimagesink ()
  {
    GstUtils::clean_element (xvimagesink_);
    if (on_error_command_ != nullptr)
      delete on_error_command_;
    
  }

  bool
  Xvimagesink::init_gpipe ()
  {

    if (!GstUtils::make_element ("bin",&sink_bin_))
      return false;
    if (!GstUtils::make_element ("queue",&queue_))
      return false;
    if (!GstUtils::make_element ("ffmpegcolorspace", &ffmpegcolorspace_))
      return false;
#if HAVE_OSX
    if (!GstUtils::make_element ("osxvideosink", &xvimagesink_))
      return false;
 #else
    if (!GstUtils::make_element ("xvimagesink", &xvimagesink_))
      return false;
#endif
   
    gst_bin_add_many (GST_BIN (sink_bin_),
		      queue_,
		      ffmpegcolorspace_,
		      xvimagesink_,
		      nullptr);
    gst_element_link_many (queue_, ffmpegcolorspace_, xvimagesink_, nullptr);

    g_object_set (G_OBJECT (xvimagesink_),
		  "draw-borders", TRUE,
		  "force-aspect-ratio", TRUE,
		  "sync", FALSE, 
		  nullptr);

    GstPad *sink_pad = gst_element_get_static_pad (queue_, 
						   "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (nullptr, sink_pad);
    gst_pad_set_active(ghost_sinkpad, TRUE);
    gst_element_add_pad (sink_bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);

    // install_property (G_OBJECT (xvimagesink_),
    // 		       "force-aspect-ratio",
    // 		       "force-aspect-ratio", 
    // 		       "Force Aspect Ratio");
    
    on_error_command_ = new QuiddityCommand ();
    on_error_command_->id_ = QuiddityCommand::remove;
    on_error_command_->add_arg (get_nick_name ());

    g_object_set_data (G_OBJECT (xvimagesink_), 
     		       "on-error-command",
     		       (gpointer)on_error_command_);
    
    set_sink_element (sink_bin_);

    return true;
  }

  bool 
  Xvimagesink::can_sink_caps (std::string caps) 
  {
    return GstUtils::can_sink_caps ("ffmpegcolorspace",
				    caps);
  };

}
