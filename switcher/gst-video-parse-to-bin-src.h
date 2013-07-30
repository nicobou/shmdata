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


#ifndef __SWITCHER_GST_VIDEO_PARSE_TO_BIN_SRC_H__
#define __SWITCHER_GST_VIDEO_PARSE_TO_BIN_SRC_H__

#include "base-source.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class GstVideoParseToBinSrc : public BaseSource
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(GstVideoParseToBinSrc);
    ~GstVideoParseToBinSrc ();
    bool to_shmdata (std::string gst_pipeline_description);

  private:
    GstElement *gst_video_parse_to_bin_src_;
    static gboolean to_shmdata_wrapped (gpointer descr, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
