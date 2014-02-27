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


#ifndef __SWITCHER_VIDEO_TEST_SOURCE_H__
#define __SWITCHER_VIDEO_TEST_SOURCE_H__

#include "video-source.h"
#include <memory>

namespace switcher
{

  class VideoTestSource : public VideoSource
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(VideoTestSource);
    VideoTestSource ();
    ~VideoTestSource ();
    VideoTestSource (const VideoTestSource &) = delete;
    VideoTestSource &operator= (const VideoTestSource &) = delete;

  private:
    GstElement *videotestsrc_;
    bool make_video_source (GstElement **new_element);
    bool on_start (); 
    bool on_stop (); 
    bool init_segment ();
  };

}  // end of namespace

#endif // ifndef
