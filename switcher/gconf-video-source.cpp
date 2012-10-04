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

#include "switcher/gconf-video-source.h"
#include <gst/gst.h>

namespace switcher
{

  GconfVideoSource::GconfVideoSource ()
  {
    make_gconfvideosource ();
  }
 
  GconfVideoSource::GconfVideoSource (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_gconfvideosource ();
  }

  void 
  GconfVideoSource::make_gconfvideosource ()
  {
    data_cond_ = g_cond_new (); 
    data_mutex_ = g_mutex_new ();
    g_main_context_invoke (NULL, (GSourceFunc) GconfVideoSource::do_init, (gpointer) this);
    g_mutex_lock (data_mutex_);
    g_cond_wait (data_cond_, data_mutex_);
    g_mutex_unlock (data_mutex_);
  }

  gboolean 
  GconfVideoSource::do_init(gpointer user_data)
  {
    GconfVideoSource *context = static_cast<GconfVideoSource *>(user_data);

    g_mutex_lock (context->data_mutex_);

    context->gconfvideosource_ = gst_element_factory_make ("gconfvideosrc",NULL);
    //set the name
    context->name_ = gst_element_get_name (context->gconfvideosource_);
    context->set_raw_video_element (context->gconfvideosource_);

    g_cond_signal (context->data_cond_);
    g_mutex_unlock (context->data_mutex_);

    return FALSE; //the source should be removed from the main loop
  }

  QuiddityDocumentation GconfVideoSource::doc_ ("video source", "gconfvideosrc",
						  "Video source embedding the GConf-settings for video input");
  
  QuiddityDocumentation 
  GconfVideoSource::get_documentation ()
  {
    return doc_;
  }

}
