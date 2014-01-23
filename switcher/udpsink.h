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


#ifndef __SWITCHER_UDPSINK_H__
#define __SWITCHER_UDPSINK_H__

#include "base-sink.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class UDPSink : public BaseSink
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(UDPSink);
    UDPSink ();
    ~UDPSink ();
    UDPSink (const UDPSink &) = delete;
    UDPSink &operator= (const UDPSink &) = delete;

    //client management
    bool remove_client (gchar *host, gint port);
    bool add_client (gchar *host, gint port);
    bool clear_clients ();

    //client management (wrapped for being invoked from the quiddity manager)
    static gboolean remove_client_wrapped (gpointer host, gint port, gpointer user_data);
    static gboolean add_client_wrapped (gpointer host, gint port, gpointer user_data);
    static gboolean clear_wrapped (gpointer unused, gpointer user_data);

  private:
    GstElement *udpsink_;
    GstElement *udpsink_bin_;
    GstElement *typefind_;
    GstPad *ghost_sinkpad_;
    static void on_client_added (GstElement *multiudpsink, gchar *host, gint port, gpointer user_data);
    static void on_client_removed (GstElement *multiudpsink, gchar *host, gint port, gpointer user_data);
    static void add_elements_to_bin (ShmdataReader *caller, void *udpbin_instance);
    bool init_segment ();
  };

}  // end of namespace

#endif // ifndef
