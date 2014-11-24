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

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

#include "./xvimagesink.hpp"
#include "./gst-utils.hpp"
#include "./quiddity-command.hpp"
#include "./scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    Xvimagesink,
    "Video Display (basic)",
    "video",
    "Video window with minimal features",
    "LGPL",
    "videosink",
    "Nicolas Bouillot");

Xvimagesink::Xvimagesink():
    sink_bin_("bin"),
    queue_("queue"),
    ffmpegcolorspace_("ffmpegcolorspace"),
#if HAVE_OSX
    xvimagesink_("osxvideosink")
#else
    xvimagesink_("xvimagesink")
#endif
{}

Xvimagesink::~Xvimagesink() {
}

bool Xvimagesink::init_gpipe() {
  if (!sink_bin_ || !queue_ || ! ffmpegcolorspace_ || !xvimagesink_)
    return false;
  gst_bin_add_many(GST_BIN(sink_bin_.get_raw()),
                   queue_.get_raw(),
                   ffmpegcolorspace_.get_raw(),
                   xvimagesink_.get_raw(),
                   nullptr);
  gst_element_link_many(queue_.get_raw(),
                        ffmpegcolorspace_.get_raw(),
                        xvimagesink_.get_raw(),
                        nullptr);
  g_object_set(G_OBJECT(xvimagesink_.get_raw()),
               "draw-borders", TRUE,
               "force-aspect-ratio", TRUE,
               "sync", FALSE,
               nullptr);
  GstPad *sink_pad = gst_element_get_static_pad(queue_.get_raw(), "sink");
  On_scope_exit{gst_object_unref(sink_pad);};
  GstPad *ghost_sinkpad = gst_ghost_pad_new(nullptr, sink_pad);
  gst_pad_set_active(ghost_sinkpad, TRUE);
  gst_element_add_pad(sink_bin_.get_raw(), ghost_sinkpad);
  g_object_set_data(G_OBJECT(xvimagesink_.get_raw()),
                    "on-error-delete",
                    (gpointer) get_nickname_cstr());
  set_sink_element(sink_bin_.get_raw());
  return true;
}

bool Xvimagesink::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("ffmpegcolorspace", caps);
}
}  // namespace switcher
