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


#ifndef __SWITCHER_AUDIO_TEST_SOURCE_H__
#define __SWITCHER_AUDIO_TEST_SOURCE_H__

#include "audio-source.h"
#include <memory>

namespace switcher
{

  class AudioTestSource : public AudioSource
  {
  public:
    typedef std::shared_ptr<AudioTestSource> ptr;
    ~AudioTestSource ();
    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private: 
   GstElement *audiotestsrc_;
  };

}  // end of namespace

#endif // ifndef
