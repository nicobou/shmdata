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
    VideoSource();

  private:
    GstElement *rawvideo_;
    GstElement *video_tee_;
    GstElement *colorspace_in_;
    GstElement *textoverlay_;
    GstElement *videoflip_;
    GstElement *colorspace_out_;

  protected:
    //called in the derived class constructor
    GstElementCleaner::ptr cleaner_;
    void set_raw_video_element (GstElement *elt);
    
  };

}  // end of namespace

#endif // ifndef
