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
    typedef std::shared_ptr<GstVideoParseToBinSrc> ptr;
    bool init ();
    bool to_shmdata (std::string gst_pipeline_description);
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private:
    GstElement *gst_video_parse_to_bin_src_;
    static gboolean to_shmdata_wrapped (gpointer descr, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
