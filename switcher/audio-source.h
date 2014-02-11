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
    AudioSource (const AudioSource &) = delete;
    AudioSource &operator= (const AudioSource &) = delete;
  private:
    GstElement *rawaudio_;
    GstElement *audio_tee_;
    GstElement *audioconvert_;   
    GstElement *resample_;
    void make_audio_elements ();
    std::string shmdata_path_;

  protected:
    void set_raw_audio_element (GstElement *elt);
    void unset_raw_audio_element ();
  };

}  // end of namespace

#endif // ifndef
