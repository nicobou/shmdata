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


#ifndef __SWITCHER_GCONF_VIDEO_SINK_H__
#define __SWITCHER_GCONF_VIDEO_SINK_H__

#include "switcher/video-sink.h"
#include <memory>

namespace switcher
{

  class GconfVideoSink : public VideoSink
  {
  public:
    typedef std::tr1::shared_ptr<GconfVideoSink> ptr;
    GconfVideoSink ();
    GconfVideoSink (QuiddityLifeManager::ptr life_manager);
   
    static QuiddityDocumentation get_documentation ();
   
  private:
    void make_gconfvideosink ();
    static QuiddityDocumentation doc_;
    GstElement *gconfvideosink_;
    GCond* data_cond_; //required in order to ensure gconf element will be factored into the main thread
    GMutex* data_mutex_;
    static gboolean do_init(gpointer user_data);
    //static void update_sub_sync_foreach (const GValue * item, gpointer data);
    static gboolean sync_sinks_wrapped (gpointer do_sync, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
