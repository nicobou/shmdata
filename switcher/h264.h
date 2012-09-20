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


#ifndef __SWITCHER_H264_H__
#define __SWITCHER_H264_H__

#include "switcher/video-sink.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class H264 : public VideoSink
  {
  public:
    typedef std::tr1::shared_ptr<H264> ptr;
    H264 ();

    static void make_shmdata_writer(void *h264_instance);

    static BaseEntityDocumentation get_documentation ();

  private:
    static BaseEntityDocumentation doc_;
    GstElement *h264_;
  };

}  // end of namespace

#endif // ifndef
