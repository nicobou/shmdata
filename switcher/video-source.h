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


#ifndef __SWITCHER_VIDEO_SOURCE_H__
#define __SWITCHER_VIDEO_SOURCE_H__

#include "base-source.h"
#include "gst-element-cleaner.h"
#include <memory>

namespace switcher
{

  class VideoSource : public BaseSource
  {
  public:
    typedef std::shared_ptr<VideoSource> ptr;
    VideoSource ();
    ~VideoSource ();
  private:
    GstElement *rawvideo_;
    GstElement *video_tee_;
    /* GstElement *colorspace_in_; */
    /* GstElement *textoverlay_; */
    /* GstElement *videoflip_; */
    /* GstElement *colorspace_out_; */
    GstCaps *videocaps_;
    void clean_video_source_elements ();
    void make_elements ();

  protected:
    //called in the derived class constructor
    GstElementCleaner::ptr cleaner_;
    void set_raw_video_element (GstElement *elt);
    
  };

}  // end of namespace

#endif // ifndef
