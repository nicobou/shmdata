/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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


#ifndef __SWITCHER_UDPSINK_H__
#define __SWITCHER_UDPSINK_H__

#include "switcher/base-sink.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class UDPSink : public BaseSink
  {
  public:
    typedef std::tr1::shared_ptr<UDPSink> ptr;
    UDPSink ();
    UDPSink (QuiddityLifeManager::ptr life_manager);

    static QuiddityDocumentation get_documentation ();

    //client management
    bool remove_client (gchar *host, gint port);
    bool add_client (gchar *host, gint port);
    bool clear_clients ();

    //client management (wrapped for being invoked from the quiddity manager)
    static gboolean remove_client_wrapped (gpointer host, gint port, gpointer user_data);
    static gboolean add_client_wrapped (gpointer host, gint port, gpointer user_data);
    static gboolean clear_wrapped (gpointer user_data);

  private:
    static QuiddityDocumentation doc_;
    GstElement *udpsink_;
    void make_udpsink ();
    static void on_client_added (GstElement *multiudpsink, gchar *host, gint port, gpointer user_data);
    static void on_client_removed (GstElement *multiudpsink, gchar *host, gint port, gpointer user_data);

  };

}  // end of namespace

#endif // ifndef
