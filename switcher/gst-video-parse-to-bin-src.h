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


#ifndef __SWITCHER_GST_VIDEO_PARSE_TO_BIN_SRC_H__
#define __SWITCHER_GST_VIDEO_PARSE_TO_BIN_SRC_H__

#include "video-source.h"
#include "custom-property-helper.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class GstVideoParseToBinSrc : public VideoSource
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(GstVideoParseToBinSrc);
    GstVideoParseToBinSrc ();
    ~GstVideoParseToBinSrc ();
    GstVideoParseToBinSrc (const GstVideoParseToBinSrc &) = delete;
    GstVideoParseToBinSrc &operator= (const GstVideoParseToBinSrc &) = delete;

    bool on_start ();
    bool on_stop ();
   
  private:
    GstElement *gst_video_parse_to_bin_src_;
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *gst_launch_pipeline_spec_;
    gchar *gst_launch_pipeline_;
    static void set_gst_launch_pipeline (const gchar *value, void *user_data);
    static const gchar *get_gst_launch_pipeline (void *user_data);
    bool make_video_source (GstElement **new_element);
    bool init_gpipe () final;
  };

}  // end of namespace

#endif // ifndef
