/*
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
    V4L2Src ();
    ~V4L2Src ();
    V4L2Src (const V4L2Src &) = delete;
    V4L2Src &operator= (const V4L2Src &) = delete;
    
    //use "NONE" for used arguments
    /* bool capture_full (const char *device_file_path,  */
    /* 		       const char *width, */
    /* 		       const char *height, */
    /* 		       const char *framerate_numerator, */
    /* 		       const char *framerate_denominator, */
    /* 		       const char *tv_standard); */
    
    static bool is_v4l_device (const char *file);
   
    bool inspect_file_device (const char *file_path);
    bool check_folder_for_v4l2_devices (const char *dir_path);

  private:
    bool on_start ();
    bool on_stop ();
    bool make_video_source (GstElement **new_element);
    
    GstElement *v4l2src_;
    GstElement *v4l2_bin_;
    GstElement *capsfilter_;

    typedef struct {
      std::string card_;
      std::string file_device_;
      std::string bus_info_;
      std::string driver_;
      std::vector <std::pair <std::string/*name*/,std::string /*description*/> > pixel_formats_; 
      std::vector <std::pair <std::string/*width*/,std::string /*height*/> > frame_size_discrete_;
      gint frame_size_stepwise_max_width_;
      gint frame_size_stepwise_min_width_;
      gint frame_size_stepwise_step_width_;
      gint frame_size_stepwise_max_height_;
      gint frame_size_stepwise_min_height_;
      gint frame_size_stepwise_step_height_;
      std::vector <std::string> tv_standards_;
      std::vector <std::pair <std::string/*numerator*/,std::string /*denominator*/> > frame_interval_discrete_;
      gint frame_interval_stepwise_min_numerator_;
      gint frame_interval_stepwise_min_denominator_;
      gint frame_interval_stepwise_max_numerator_;
      gint frame_interval_stepwise_max_denominator_;
      gint frame_interval_stepwise_step_numerator_;
      gint frame_interval_stepwise_step_denominator_;
    } CaptureDescription;

    bool make_elements ();
    void clean_elements ();
    void update_capture_device ();
    void update_device_specific_properties (gint device_enum_id);
    void update_discrete_resolution (CaptureDescription descr);
    void update_width_height (CaptureDescription descr);
    void update_tv_standard (CaptureDescription descr);
    void update_discrete_framerate (CaptureDescription cap_descr);
    void update_framerate_numerator_denominator (CaptureDescription cap_descr);

    /* static gboolean capture_full_wrapped (gpointer device_file_path,  */
    /* 					  gpointer width, */
    /* 					  gpointer height, */
    /* 					  gpointer framerate_numerator, */
    /* 					  gpointer framerate_denominator, */
    /* 					  gpointer tv_standard, */
    /* 					  gpointer user_data); */

    /* static gboolean capture_wrapped (gpointer device_file_path,  */
    /* 				     gpointer user_data); */
    
    static std::string pixel_format_to_string (unsigned pf_id);
    static bool inspect_frame_rate (const char *file_path,
				    unsigned pixel_format,
				    unsigned width,
				    unsigned height);
    static const gchar *get_capture_devices_json (void *user_data);
    
    //custom properties:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *capture_devices_description_spec_;//json formated
    gchar *capture_devices_description_;//json formated
    
    //device enum and select
    GParamSpec *devices_enum_spec_;
    GEnumValue devices_enum_[128];
    gint device_;
    static void set_camera (const gint value, void *user_data);
    static gint get_camera (void *user_data);
    
    //resolution enum and select for the currently selected device,
    //this is updated when selecting an other device
    GParamSpec *resolutions_spec_; 
    GEnumValue resolutions_enum_ [128];
    gint resolution_;
    static void set_resolution (const gint value, void *user_data);
    static gint get_resolution (void *user_data);

    //width height for the currently selected device
    GParamSpec *width_spec_;
    GParamSpec *height_spec_;
    gint width_;
    gint height_;
    static void set_width (const gint value, void *user_data);
    static gint get_width (void *user_data);
    static void set_height (const gint value, void *user_data);
    static gint get_height (void *user_data);

    //tv standard enum and select for the currently selected device,
    //this is updated when selecting an other device
    GParamSpec *tv_standards_spec_; 
    GEnumValue tv_standards_enum_ [128];
    gint tv_standard_;
    static void set_tv_standard (const gint value, void *user_data);
    static gint get_tv_standard (void *user_data);

    //framerate enum and select for the currently selected device,
    //this is updated when selecting an other device
    GParamSpec *framerate_spec_; 
    GEnumValue framerates_enum_ [128];
    gint framerate_;
    static void set_framerate (const gint value, void *user_data);
    static gint get_framerate (void *user_data);

    //width height for the currently selected device
    GParamSpec *framerate_numerator_spec_;
    GParamSpec *framerate_denominator_spec_;
    gint framerate_numerator_;
    gint framerate_denominator_;
    static void set_framerate_numerator (const gint value, void *user_data);
    static gint get_framerate_numerator (void *user_data);
    static void set_framerate_denominator (const gint value, void *user_data);
    static gint get_framerate_denominator (void *user_data);

    std::vector <CaptureDescription> capture_devices_; //FIXME should be static

    bool init_segment ();
  };
  
  SWITCHER_DECLARE_PLUGIN(V4L2Src);

}  // end of namespace

#endif // ifndef
