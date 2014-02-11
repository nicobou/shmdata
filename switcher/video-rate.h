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


#ifndef __SWITCHER_VIDEO_RATE_H__
#define __SWITCHER_VIDEO_RATE_H__

#include "base-sink.h"
#include "gst-element-cleaner.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{
  class VideoRate : public BaseSink, public GstElementCleaner
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(VideoRate);
    VideoRate ();
    VideoRate (const VideoRate &) = delete;
    VideoRate &operator= (const VideoRate &) = delete;
    static void make_shmdata_writer(ShmdataReader *caller, void *vorbis_instance);

  private:
    GstElement *video_rate_bin_;
    GstElement *video_rate_enc_;
    bool init_segment ();
  };

}  // end of namespace

#endif // ifndef
