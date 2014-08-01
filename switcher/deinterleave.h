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


#ifndef __SWITCHER_DEINTERLEAVE_H__
#define __SWITCHER_DEINTERLEAVE_H__

#include "single-pad-gst-sink.h"
#include <memory>
#include <map>

namespace switcher
{

  class Deinterleave : public SinglePadGstSink, public GstElementCleaner
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(Deinterleave);
    Deinterleave ();
    Deinterleave (const Deinterleave &) = delete;
    Deinterleave &operator= (const Deinterleave &) = delete;

  private: 
   GstElement *deinterleave_;
   std::map<std::string, int> media_counters_;

   bool init_gpipe () final;
   bool can_sink_caps (std::string caps) final;

   static void make_deinterleave_active (ShmdataReader *caller, void *deinterleave_instance);
   static void pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data);
   static void no_more_pads_cb (GstElement* object, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
