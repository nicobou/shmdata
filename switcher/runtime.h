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

/**
 * The Runtime class
 */

#ifndef __SWITCHER_RUNTIME_H__
#define __SWITCHER_RUNTIME_H__

#include <gst/gst.h>
#include "quiddity.h"
#include <memory>

namespace switcher
{
  
  class Runtime : public Quiddity
    {
    public:
      typedef std::shared_ptr<Runtime> ptr;
      ~Runtime ();
      bool init ();
      bool play ();
      bool pause ();
      bool seek (gdouble position);
      bool speed (gdouble speed);

      QuiddityDocumentation get_documentation ();
      static QuiddityDocumentation doc_;
      GstElement *get_pipeline ();
      static gboolean play_wrapped (gpointer unused, gpointer user_data);
      static gboolean pause_wrapped (gpointer unused, gpointer user_data);
      static gboolean seek_wrapped (gdouble position, gpointer user_data);
      static gboolean speed_wrapped (gdouble speed, gpointer user_data);
    private:
     typedef struct {
       Runtime *self;
       gchar *name;
     } QuidRemoveArgs;
      GstElement *pipeline_;
      guint64 speed_ ;
      static gboolean bus_called (GstBus *bus, GstMessage *msg, gpointer data); 
      static GstBusSyncReply bus_sync_handler (GstBus *bus, GstMessage *msg, gpointer user_data);
      static gboolean remove_quid (gpointer user_data);
      //GstBus is a specific context:
      typedef struct
      {
	GSource source;
	GstBus *bus;
	gboolean inited;

      } GstBusSource;
      static gboolean source_prepare(GSource *source, 
				     gint *timeout);
      static gboolean source_check(GSource *source);
      static gboolean source_dispatch(GSource *source, 
				      GSourceFunc callback,
				      gpointer user_data);
      static void source_finalize (GSource * source);
      GSourceFuncs source_funcs_;
      GSource *source_;
    };

  
} // end of namespace

#endif // ifndef
