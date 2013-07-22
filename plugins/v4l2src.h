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


#ifndef __SWITCHER_V4L2SRC_H__
#define __SWITCHER_V4L2SRC_H__

#include "switcher/video-source.h"
#include "switcher/custom-property-helper.h"
#include <memory>


namespace switcher
{
  
  class V4L2Src : public VideoSource 
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(V4L2Src);

    ~V4L2Src ();
  
    //use "NONE" for used arguments
    bool capture (const char *device_file_path, 
		  const char *width,
		  const char *height,
		  const char *framerate_numerator,
		  const char *framerate_denominator,
		  const char *tv_standard);
    
    static bool is_v4l_device (const char *file);
   
    bool inspect_file_device (const char *file_path);
    bool check_folder_for_v4l2_devices (const char *dir_path);

  private:
    GstElement *v4l2src_;
    static gboolean capture_wrapped (gpointer device_file_path, 
				     gpointer width,
				     gpointer height,
				     gpointer framerate_numerator,
				     gpointer framerate_denominator,
				     gpointer tv_standard,
				     gpointer user_data);
    static std::string pixel_format_to_string (unsigned pf_id);
    static bool inspect_frame_rate (const char *file_path,
			     unsigned pixel_format,
			     unsigned width,
			     unsigned height);
    static gchar *get_capture_devices_json (void *user_data);

    //custom properties:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *capture_devices_description_spec_;//json formated
    gchar *capture_devices_description_;//json formated
    

    typedef struct {
      std::string card_;
      std::string file_device_;
      std::string bus_info_;
      std::string driver_;
      std::vector <std::pair <std::string/*name*/,std::string /*description*/> > pixel_formats_; 
      std::vector <std::pair <std::string/*width*/,std::string /*height*/> > frame_size_discrete_;
      std::string frame_size_stepwise_max_width_;
      std::string frame_size_stepwise_min_width_;
      std::string frame_size_stepwise_step_width_;
      std::string frame_size_stepwise_max_height_;
      std::string frame_size_stepwise_min_height_;
      std::string frame_size_stepwise_step_height_;
      std::vector <std::string> tv_standards_;
      std::vector <std::pair <std::string/*numerator*/,std::string /*denominator*/> > frame_interval_discrete_;
      std::string frame_interval_stepwise_min_numerator_;
      std::string frame_interval_stepwise_min_denominator_;
      std::string frame_interval_stepwise_max_numerator_;
      std::string frame_interval_stepwise_max_denominator_;
      std::string frame_interval_stepwise_step_numerator_;
      std::string frame_interval_stepwise_step_denominator_;
    } CaptureDescription;

    std::map <std::string, CaptureDescription> capture_devices_; //indexed by device_file_path
  };
  
  
  SWITCHER_DECLARE_PLUGIN(V4L2Src);

}  // end of namespace

#endif // ifndef
