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
 * The GPipe class
 */

#ifndef __SWITCHER_GPIPE_H__
#define __SWITCHER_GPIPE_H__

#include <gst/gst.h>
#include <memory>
#include <vector>
#include "quiddity.h"
#include "segment.h"

namespace switcher
{
  class Quiddity;
  class QuiddityCommand;
  class CustomPropertyHelper;
  class DecodebinToShmdata;

  class GPipe : public Quiddity, public Segment 
    {
      friend DecodebinToShmdata; 
    public:
      GPipe ();
      virtual ~GPipe ();
      GPipe (const GPipe &) = delete;
      GPipe &operator= (const GPipe &) = delete;
      bool init () final;
      virtual bool init_gpipe () = 0;
      
    protected:
      //void init_gpipe (Quiddity &quiddity);//FIXME should called quiddity-manager-impl 
      //(privite with manager-impl friend ? dynamic cast ?) this will avoid to invoke init_startable (this)
      GstElement *get_bin ();
      GstElement *bin_ {nullptr}; //FIXME should be private
      bool reset_bin ();
      GstElement *get_pipeline ();
      void install_play_pause ();
      void install_seek ();
      void install_speed ();
      void play (gboolean);
      bool seek (gdouble position_in_ms);
      void query_position_and_length ();

    private:
     typedef struct {
       GPipe *self;
       QuiddityCommand *command;
       GSource *src;
     } QuidCommandArg;
      //GstBus is a specific context:
      typedef struct
      {
	GSource source;
	GstBus *bus;
	gboolean inited;
      } GstBusSource;

      GstElement *pipeline_ {nullptr};
      gdouble speed_ {1.0};
      GSource *position_tracking_source_ {nullptr};
      GSourceFuncs source_funcs_;
      GSource *source_{nullptr};
      std::shared_ptr<CustomPropertyHelper> gpipe_custom_props_;
      GParamSpec *play_pause_spec_ {nullptr};
      bool play_ {true};
      GParamSpec *seek_spec_ {nullptr};
      gdouble seek_ {0.0};
      gint64 length_ {0};
      std::vector<GSource *> commands_ {};
 
      void make_bin ();
      void clean_bin ();
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
