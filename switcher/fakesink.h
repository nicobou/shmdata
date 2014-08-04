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


#ifndef __SWITCHER_FAKESINK_H__
#define __SWITCHER_FAKESINK_H__

#include "single-pad-gst-sink.h"
#include "gst-element-cleaner.h"
#include "custom-property-helper.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class FakeSink : public SinglePadGstSink
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(FakeSink);
    FakeSink ();
    ~FakeSink ();
    FakeSink (const FakeSink &) = delete;
    FakeSink &operator= (const FakeSink &) = delete;

  private:
    GstElement *fakesink_;
    gsize num_bytes_since_last_update_;
    GSource *update_byterate_source_;
    gint byte_rate_;
    gchar *string_caps_;
    gboolean set_string_caps_;
    //byte rate property 
    CustomPropertyHelper::ptr props_;
    GParamSpec *byte_rate_spec_;
    GParamSpec *caps_spec_;

    bool init_gpipe () final;
    bool can_sink_caps (std::string caps) final {return true;};

    static void on_handoff_cb (GstElement* object,
			       GstBuffer* buf,
			       GstPad* pad,
			       gpointer user_data);
    static gboolean update_byte_rate (gpointer user_data); 
    static gint get_byte_rate (void *user_data);
    static const gchar *get_caps (void *user_data);
  };

}  // end of namespace

#endif // ifndef
