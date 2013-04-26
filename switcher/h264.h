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


#ifndef __SWITCHER_H264_H__
#define __SWITCHER_H264_H__

#include "video-sink.h"
#include "gst-element-cleaner.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class H264 : public VideoSink, public GstElementCleaner
  {
  public:
    typedef std::shared_ptr<H264> ptr;

    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;
    
    static void make_shmdata_writer(ShmdataReader *caller, void *h264_instance);

  private:
    GstElement *h264bin_;
    GstElement *h264enc_;
  };

}  // end of namespace

#endif // ifndef
