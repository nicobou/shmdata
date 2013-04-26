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


#ifndef __SWITCHER_AUDIO_SOURCE_H__
#define __SWITCHER_AUDIO_SOURCE_H__

#include "base-source.h"
#include <memory>


namespace switcher
{
  class AudioSource : public BaseSource
  {
  public:
    typedef std::shared_ptr<AudioSource> ptr;
    AudioSource ();

  private:
    GstElement *rawaudio_;
    GstElement *audio_tee_;
    //GstElement *audio_queue_;
    GstElement *audioconvert_;   
    GstElement *pitch_;
    GstElement *resample_;

  protected:
    //called in the derived class constructor
    GstElementCleaner::ptr cleaner_;
    void set_raw_audio_element (GstElement *elt);
  };

}  // end of namespace

#endif // ifndef
