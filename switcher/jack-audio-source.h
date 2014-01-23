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


#ifndef __SWITCHER_JACK_AUDIO_SOURCE_H__
#define __SWITCHER_JACK_AUDIO_SOURCE_H__

#include "audio-source.h"
#include <memory>

namespace switcher
{

  class JackAudioSource : public AudioSource
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(JackAudioSource);
    JackAudioSource ();
    ~JackAudioSource ();
    JackAudioSource (const JackAudioSource&) = delete;
    JackAudioSource &operator= (const JackAudioSource&) = delete;

  private: 
   GstElement *jackaudiosrc_;
   bool init_segment ();
  };

}  // end of namespace

#endif // ifndef
