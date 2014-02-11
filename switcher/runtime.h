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

/**
 * The Runtime class
 */

#ifndef __SWITCHER_RUNTIME_H__
#define __SWITCHER_RUNTIME_H__

#include <gst/gst.h>
#include <memory>

namespace switcher
{
  class Quiddity;
  class QuiddityCommand;
  class CustomPropertyHelper;

  class Runtime
    {
    public:
      Runtime ();
      virtual ~Runtime ();
      Runtime (const Runtime &) = delete;
      Runtime &operator= (const Runtime &) = delete;

    protected:
      void init_runtime (Quiddity &quiddity);//FIXME should called quiddity-manager-impl 
      //(privite with manager-impl friend ? dynamic cast ?) this will avoid to invoke init_startable (this)
      GstElement *get_pipeline ();
      void install_play_pause ();
      void install_seek ();
      void install_speed ();
      void play (gboolean);
      bool seek (gdouble position_in_ms);
      void query_position_and_length ();

    private:
     typedef struct {
       Runtime *self;
       QuiddityCommand *command;
     } QuidCommandArg;
      //GstBus is a specific context:
      typedef struct
      {
	GSource source;
	GstBus *bus;
	gboolean inited;
      } GstBusSource;

      GstElement *pipeline_;
      gdouble speed_;//was gunint64 ???
      GSource *position_tracking_source_;
      GSourceFuncs source_funcs_;
      GSource *source_;
      Quiddity *quid_;
      std::shared_ptr<CustomPropertyHelper> custom_props_;
      GParamSpec *play_pause_spec_;
      bool play_;
      GParamSpec *seek_spec_;
      gdouble seek_;
      gint64 length_;

      bool speed (gdouble speed);
      static gboolean get_play (void *user_data);
      static void set_play (gboolean play, void *user_data);
      static gdouble get_seek (void *user_data);
      static void set_seek (gdouble position, void *user_data);
      static gboolean speed_wrapped (gdouble speed, gpointer user_data);
      static gboolean bus_called (GstBus *bus, GstMessage *msg, gpointer data); 
      static GstBusSyncReply bus_sync_handler (GstBus *bus, GstMessage *msg, gpointer user_data);
      static gboolean run_command (gpointer user_data);
      static gboolean source_prepare(GSource *source, 
				     gint *timeout);
      static gboolean source_check(GSource *source);
      static gboolean source_dispatch(GSource *source, 
				      GSourceFunc callback,
				      gpointer user_data);
      static void source_finalize (GSource * source);
      static void print_one_tag (const GstTagList *list, 
				 const gchar *tag, 
				 gpointer user_data);
      static gboolean query_position (gpointer user_data);
    };
} // end of namespace

#endif // ifndef
