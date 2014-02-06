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
#include "startable-quiddity.h"
#include <memory>

namespace switcher
{

  class VideoSource : public BaseSource, public StartableQuiddity 
  {
  public:
    typedef std::shared_ptr<VideoSource> ptr;
    VideoSource ();
    ~VideoSource ();
    VideoSource (const VideoSource &) = delete;
    VideoSource &operator= (const VideoSource&) = delete;
    bool start ();
    bool stop ();

  private:
    GstElement *rawvideo_;
    GstElement *video_tee_;
    GstCaps *videocaps_;
    std::string shmdata_path_;
    //custom properties:
    CustomPropertyHelper::ptr custom_props_; 
    //codec //FIXME make this static 
    GParamSpec *primary_codec_spec_;
    GEnumValue primary_codec_[128];
    GParamSpec *secondary_codec_spec_;
    GEnumValue secondary_codec_[128];
    gint codec_;
    //short or long codec list
    GParamSpec *codec_long_list_spec_;
    bool codec_long_list_;
    GstElement *codec_element_;
    GstElement *queue_codec_element_;
    GstElement *color_space_codec_element_;
    std::vector<std::string> codec_properties_;

    virtual bool on_start () {return true;};
    virtual bool on_stop () {return true;};
    virtual bool make_video_source (GstElement **new_element) = 0;
    bool make_new_shmdatas ();
    bool remake_codec_elements ();
    void make_codec_properties ();
    static void set_codec (const gint value, void *user_data);
    static gint get_codec (void *user_data);
    static gboolean get_codec_long_list (void *user_data);
    static void set_codec_long_list (gboolean mute, void *user_data);
    static gboolean sink_factory_filter (GstPluginFeature * feature, gpointer data);
    static gint sink_compare_ranks (GstPluginFeature * f1, GstPluginFeature * f2);
    static void print_list (gpointer data, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
