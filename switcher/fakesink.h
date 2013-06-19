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


#ifndef __SWITCHER_FAKESINK_H__
#define __SWITCHER_FAKESINK_H__

#include "base-sink.h"
#include "gst-element-cleaner.h"
#include "custom-property-helper.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class FakeSink : public BaseSink
  {
  public:
    typedef std::shared_ptr<FakeSink> ptr;
    ~FakeSink ();
    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private:
    GstElement *fakesink_;
    gsize num_bytes_since_last_update_;
    //guint update_byterate_id_; 
    GSource *update_byterate_source_;
    gint byte_rate_;
    gchar *string_caps_;
    gboolean set_string_caps_;
    static void on_handoff_cb (GstElement* object,
			       GstBuffer* buf,
			       GstPad* pad,
			       gpointer user_data);
    static gboolean update_byte_rate (gpointer user_data); 
    static gint get_byte_rate (void *user_data);
    static gchar * get_caps (void *user_data);
    //byte rate property 
    CustomPropertyHelper::ptr props_;
    GParamSpec *byte_rate_spec_;
    GParamSpec *caps_spec_;

  };

}  // end of namespace

#endif // ifndef
