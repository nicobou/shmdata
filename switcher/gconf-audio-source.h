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


#ifndef __SWITCHER_GCONF_AUDIO_SOURCE_H__
#define __SWITCHER_GCONF_AUDIO_SOURCE_H__

#include "switcher/audio-source.h"
#include <memory>

namespace switcher
{

  class GconfAudioSource : public AudioSource
  {
  public:
    typedef std::shared_ptr<GconfAudioSource> ptr;
    GconfAudioSource ();
    GconfAudioSource (QuiddityLifeManager::ptr life_manager);

    QuiddityDocumentation get_documentation ();
    static const QuiddityDocumentation doc_;

  private:
    void make_gconfaudiosource ();
    GstElement *gconfaudiosource_;
    static gboolean do_init(gpointer user_data);
    GCond* data_cond_; //required in order to ensure gconf element will be factored into the main thread
    GMutex* data_mutex_;
  };

}  // end of namespace

#endif // ifndef
