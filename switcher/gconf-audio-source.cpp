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

#include "switcher/gconf-audio-source.h"
#include <gst/gst.h>

namespace switcher
{


  bool 
  GconfAudioSource::init ()
  {
    data_cond_ = g_cond_new (); 
    data_mutex_ = g_mutex_new ();
    g_main_context_invoke (NULL, (GSourceFunc) GconfAudioSource::do_init, (gpointer) this);
    g_mutex_lock (data_mutex_);
    g_cond_wait (data_cond_, data_mutex_);
    g_mutex_unlock (data_mutex_);
    return true;
  }

  gboolean 
  GconfAudioSource::do_init(gpointer user_data)
  {
    GconfAudioSource *context = static_cast<GconfAudioSource *>(user_data);

    g_mutex_lock (context->data_mutex_);

    context->gconfaudiosource_ = gst_element_factory_make ("gconfaudiosrc",NULL);
    //set the name
    context->set_name (gst_element_get_name (context->gconfaudiosource_));
    context->set_raw_audio_element (context->gconfaudiosource_);

    g_cond_signal (context->data_cond_);
    g_mutex_unlock (context->data_mutex_);

    return FALSE; //the source should be removed from the main loop
  }

  const QuiddityDocumentation GconfAudioSource::doc_ ("audio source", "gconfaudiosrc",
						  "Audio source embedding the GConf-settings for audio input");
  
  QuiddityDocumentation 
  GconfAudioSource::get_documentation ()
  {
    return doc_;
  }

}
