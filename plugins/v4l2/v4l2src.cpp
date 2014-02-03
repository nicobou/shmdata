/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-v4l2.
 *
 * switcher-v4l2 is free software: you can redistribute it and/or modify
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

#include "v4l2src.h"
#include "switcher/gst-utils.h"
#include <cstdlib>  // For srand() and rand()
#include <ctime>    // For time()

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>


namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(V4L2Src,
				       "Video Capture (with v4l2)",
				       "video source", 
				       "Discover and use v4l2 supported capture cards and cameras",
				       "GPL",
				       "v4l2src",				
				       "Nicolas Bouillot");
  
  V4L2Src::V4L2Src () :
    v4l2src_ (NULL),
    v4l2_bin_ (NULL),
    capsfilter_ (NULL),
    custom_props_ (new CustomPropertyHelper ()), 
    capture_devices_description_spec_ (NULL),
    capture_devices_description_ (NULL),
    devices_enum_spec_ (NULL),
    devices_enum_ (),
    device_ (0),
    resolutions_spec_ (NULL), 
    resolutions_enum_ (),
    resolution_ (0),
    width_spec_ (NULL),
    height_spec_ (NULL),
    width_ (0),
    height_ (0),
    tv_standards_spec_ (NULL), 
    tv_standards_enum_ (),
    tv_standard_ (0),
    framerate_spec_ (NULL), 
    framerates_enum_ (),
    framerate_ (0),
    framerate_numerator_spec_ (NULL),
    framerate_denominator_spec_ (NULL),
    framerate_numerator_ (0),
    framerate_denominator_ (1),
    capture_devices_ ()
  {}

  bool
  V4L2Src::init_segment ()
  {
    if (!make_elements ())
      return false;

    //device inspector
    check_folder_for_v4l2_devices ("/dev");
    update_capture_device ();
    
    if (capture_devices_.empty ())
      {
	g_debug ("no video 4 linux device detected");
	return false;
      }
    capture_devices_description_spec_ = 
      custom_props_->make_string_property ("devices-json", 
					   "Description of capture devices (json formated)",
					   get_capture_devices_json (this),
					   (GParamFlags) G_PARAM_READABLE,
					   NULL,
					   V4L2Src::get_capture_devices_json,
					   this);
      
      install_property_by_pspec (custom_props_->get_gobject (), 
				  capture_devices_description_spec_, 
				  "devices-json",
				  "Capture Devices");
      
        
      devices_enum_spec_ = 
	custom_props_->make_enum_property ("device", 
					   "Enumeration of v4l2 capture devices",
					   device_, 
					   devices_enum_,
					   (GParamFlags) G_PARAM_READWRITE,
					   V4L2Src::set_camera,
					   V4L2Src::get_camera,
					   this);

      install_property_by_pspec (custom_props_->get_gobject (), 
				  devices_enum_spec_, 
				  "device",
				  "Capture Device");

      update_device_specific_properties (device_);
      return true;
  }

  void
  V4L2Src::update_capture_device ()
  {
      gint i = 0;
      for (auto &it : capture_devices_)
	{
	  devices_enum_ [i].value = i;
	  //FIXME previous free here
	  devices_enum_ [i].value_name = g_strdup (it.card_.c_str ());
	  devices_enum_ [i].value_nick = g_strdup (it.file_device_.c_str ());
	  i ++;
	}
      devices_enum_[i].value = 0;
      devices_enum_[i].value_name = NULL;
      devices_enum_[i].value_nick = NULL;
  }


  void 
  V4L2Src::update_device_specific_properties (gint device_enum_id)
  {
    if (capture_devices_.empty ())
      return;
    CaptureDescription cap_descr = capture_devices_.at (device_enum_id);

    update_discrete_resolution (cap_descr);
    update_width_height (cap_descr);
    update_tv_standard (cap_descr);
    update_discrete_framerate (cap_descr);
    update_framerate_numerator_denominator (cap_descr);
  }

  void
  V4L2Src::update_discrete_resolution (CaptureDescription cap_descr)
  {
    uninstall_property ("resolution");
    resolution_ = -1;
    if (!cap_descr.frame_size_discrete_.empty ())
      {
     	gint i = 0;
     	for (auto &it : cap_descr.frame_size_discrete_)
	  {
	    resolutions_enum_ [i].value = i;
	    //FIXME free previous here
	    resolutions_enum_ [i].value_name = g_strdup_printf ("%sx%s", 
								it.first.c_str (),
								it.second.c_str ());
	    resolutions_enum_ [i].value_nick = resolutions_enum_ [i].value_name; 
	    i ++;
	  }
     	resolutions_enum_ [i].value = 0;
     	resolutions_enum_ [i].value_name = NULL;
     	resolutions_enum_ [i].value_nick = NULL;
	
	if (resolutions_spec_ == NULL)
	  resolutions_spec_ = custom_props_->make_enum_property ("resolution", 
								 "resolution of selected capture devices",
								 0, 
								 resolutions_enum_,
								 (GParamFlags) G_PARAM_READWRITE,
								 V4L2Src::set_resolution,
								 V4L2Src::get_resolution,
								 this); 
	resolution_ = 0;
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    resolutions_spec_, 
     				    "resolution",
     				    "Resolution");
	
      }

  }

  void
  V4L2Src::update_discrete_framerate (CaptureDescription cap_descr)
  {
    uninstall_property ("framerate");
    framerate_ = -1;
    if (!cap_descr.frame_size_discrete_.empty ())
      {
     	gint i = 0;
     	for (auto &it : cap_descr.frame_interval_discrete_)
	  {
	    framerates_enum_ [i].value = i;
	    //FIXME free previous here
	    //inversing enumerator and denominator because gst wants framerate while v4l2 gives frame interval 
	    framerates_enum_ [i].value_name = g_strdup_printf ("%s/%s", 
							       it.second.c_str (),
							       it.first.c_str ());
	    framerates_enum_ [i].value_nick = framerates_enum_ [i].value_name; 
	    i ++;
	  }
     	framerates_enum_ [i].value = 0;
     	framerates_enum_ [i].value_name = NULL;
     	framerates_enum_ [i].value_nick = NULL;
	
	if (framerate_spec_ == NULL)
	  framerate_spec_ = custom_props_->make_enum_property ("framerate", 
								 "framerate of selected capture devices",
								 0, 
								 framerates_enum_,
								 (GParamFlags) G_PARAM_READWRITE,
								 V4L2Src::set_framerate,
								 V4L2Src::get_framerate,
								 this); 
	framerate_ = 0;
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    framerate_spec_, 
     				    "framerate",
     				    "Framerate");
	
      }

  }

  void
  V4L2Src::update_width_height (CaptureDescription cap_descr)
  {
    uninstall_property ("width");
    uninstall_property ("height");
    width_ = -1;
    height_ = -1;
    if (cap_descr.frame_size_stepwise_max_width_ > 0)
      {
	width_ = cap_descr.frame_size_stepwise_max_width_;	
	if (width_spec_ == NULL)
	  width_spec_ = 
	    custom_props_->make_int_property ("width", 
					      "width of selected capture devices",
					      cap_descr.frame_size_stepwise_min_width_, 
					      cap_descr.frame_size_stepwise_max_width_, 
					      cap_descr.frame_size_stepwise_max_width_, 
					      (GParamFlags) G_PARAM_READWRITE,
					      V4L2Src::set_width,
					      V4L2Src::get_width,
					      this); 
	
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    width_spec_, 
     				    "width",
     				    "Width");

	height_ = cap_descr.frame_size_stepwise_max_height_;

	if (height_spec_ == NULL)
	  height_spec_ = 
	    custom_props_->make_int_property ("height", 
					      "height of selected capture devices",
					      cap_descr.frame_size_stepwise_min_height_, 
					      cap_descr.frame_size_stepwise_max_height_, 
					      cap_descr.frame_size_stepwise_max_height_, 
					      (GParamFlags) G_PARAM_READWRITE,
					      V4L2Src::set_height,
					      V4L2Src::get_height,
					      this); 
	
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    height_spec_, 
     				    "height",
     				    "Height");
	
      }

  }

  void
  V4L2Src::update_framerate_numerator_denominator (CaptureDescription cap_descr)
  {
    uninstall_property ("framerate_numerator");
    uninstall_property ("framerate_denominator");
    framerate_numerator_ = -1;
    framerate_denominator_ = -1;
    if (cap_descr.frame_interval_stepwise_max_numerator_ > 0)
      {

	framerate_numerator_ = 60;	
	
	if (framerate_numerator_spec_ == NULL)
	  framerate_numerator_spec_ = 
	    custom_props_->make_int_property ("framerate_numerator", 
					      "framerate numerator of selected capture devices",
					      1, //FIXME do actually use values
					      60,
					      60,
					      (GParamFlags) G_PARAM_READWRITE,
					      V4L2Src::set_framerate_numerator,
					      V4L2Src::get_framerate_numerator,
					      this); 
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    framerate_numerator_spec_, 
     				    "framerate_numerator",
     				    "Framerate Numerator");

	framerate_denominator_ = 1;
	if (framerate_denominator_spec_ == NULL)
	  framerate_denominator_spec_ = 
	    custom_props_->make_int_property ("framerate_denominator", 
					      "Framerate denominator of selected capture devices",
					      1,
					      1,
					      1,
					      (GParamFlags) G_PARAM_READWRITE,
					      V4L2Src::set_framerate_denominator,
					      V4L2Src::get_framerate_denominator,
					      this); 
	
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    framerate_denominator_spec_, 
     				    "framerate_denominator",
     				    "Framerate Denominator");
	
      }

  }

  void
  V4L2Src::update_tv_standard (CaptureDescription cap_descr)
  {
    uninstall_property ("tv_standard");
    tv_standard_ = -1;
    if (cap_descr.tv_standards_.size () > 1)
      {
     	gint i = 0;
     	for (auto &it : cap_descr.tv_standards_)
	  {
	    tv_standards_enum_ [i].value = i;
	    //FIXME free previous here
	    tv_standards_enum_ [i].value_name = g_strdup (it.c_str ());
	    tv_standards_enum_ [i].value_nick = tv_standards_enum_ [i].value_name; 
	    i ++;
	  }
     	tv_standards_enum_ [i].value = 0;
     	tv_standards_enum_ [i].value_name = NULL;
     	tv_standards_enum_ [i].value_nick = NULL;
	
	if (tv_standards_spec_ == NULL)
	  tv_standards_spec_ = custom_props_->make_enum_property ("tv_standard", 
								  "tv standard of selected capture devices",
								  0, 
								  tv_standards_enum_,
								  (GParamFlags) G_PARAM_READWRITE,
								  V4L2Src::set_tv_standard,
								  V4L2Src::get_tv_standard,
								  this); 
	
	tv_standard_ = 0;
     	install_property_by_pspec (custom_props_->get_gobject (), 
     				    tv_standards_spec_, 
     				    "tv_standard",
     				    "TV Standard");
	
      }

  }

  V4L2Src::~V4L2Src ()
  {
    if (capture_devices_description_ != NULL)
      g_free (capture_devices_description_);
    clean_elements ();
  }

  bool
  V4L2Src::make_elements ()
  {
    clean_elements ();

    if (!GstUtils::make_element ("v4l2src",&v4l2src_))
      return false;
    if (!GstUtils::make_element ("capsfilter",&capsfilter_))
      return false;
    if (!GstUtils::make_element ("bin",&v4l2_bin_))
      return false;

    gst_bin_add_many (GST_BIN (v4l2_bin_),
		      v4l2src_,
		      capsfilter_,
		      NULL);

    gst_element_link (v4l2src_, capsfilter_);

    GstPad *src_pad = gst_element_get_static_pad (capsfilter_, "src");
    GstPad *ghost_srcpad = gst_ghost_pad_new (NULL, src_pad);
    gst_pad_set_active(ghost_srcpad,TRUE);
    gst_element_add_pad (v4l2_bin_, ghost_srcpad); 
    gst_object_unref (src_pad);
    return true;
  }

  void
  V4L2Src::clean_elements ()
  {
    reset_bin ();
  }

  std::string
  V4L2Src::pixel_format_to_string (unsigned pf_id)
  {
    std::string pixfmt;
    pixfmt += (char)(pf_id & 0xff);
    pixfmt += (char)((pf_id >> 8) & 0xff);
    pixfmt += (char)((pf_id >> 16) & 0xff);
    pixfmt += (char)((pf_id >> 24) & 0xff);
    return pixfmt;
  }

  bool
  V4L2Src::inspect_file_device (const char *file_path)
  {
     int fd = open(file_path, O_RDWR);

     if (fd < 0)
       {
	 g_debug ("V4L2Src: inspecting file gets negative file descriptor");
	 return false;
       }

     CaptureDescription description;

     struct v4l2_capability vcap;
     ioctl(fd, VIDIOC_QUERYCAP, &vcap);
     
     description.file_device_= file_path;
     description.card_ = (char *)vcap.card;
     description.bus_info_ = (char *)vcap.bus_info;
     description.driver_ = (char *)vcap.driver;
     
     // g_print ("-------------------------- card %s bus %s driver %s\n", 
     //  	      (char *)vcap.card,
     //  	      (char *)vcap.bus_info,
     //  	      (char *)vcap.driver);
     
     //pixel format
     v4l2_fmtdesc fmt;
     unsigned default_pixel_format = 0;
     memset(&fmt, 0, sizeof(fmt));
     fmt.index = 0;
     fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) >= 0)
       {
     	if (fmt.pixelformat != 0)
     	  {
	    if (default_pixel_format == 0) 
	    default_pixel_format = fmt.pixelformat;  
	     // g_print ("******** pixel format  %s - %s\n", 
	     // 	     pixel_format_to_string(fmt.pixelformat).c_str (), 
	     // 	     (const char *)fmt.description);
	    description.pixel_formats_.push_back (std::make_pair (pixel_format_to_string(fmt.pixelformat),
						      (const char *)fmt.description));
	  }
     	fmt.index ++;
       }

     if (default_pixel_format == 0)
       {
	 g_debug ("no default pixel format found for %s, returning",
		  file_path);
	 return false;
       }

     v4l2_frmsizeenum frmsize;
     memset(&frmsize, 0, sizeof(frmsize));
     frmsize.pixel_format = default_pixel_format;
     frmsize.index = 0;
     unsigned default_width = 0;
     unsigned default_height = 0;
     while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0 && frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
       {
     	if (frmsize.index == 0)
     	  {
     	    default_width = frmsize.discrete.width;
     	    default_height = frmsize.discrete.height;
     	  }
	char *width = g_strdup_printf ("%u",frmsize.discrete.width);
	char *height = g_strdup_printf ("%u",frmsize.discrete.height);
	description.frame_size_discrete_.push_back (std::make_pair (width, height));
	g_free (width);
	g_free (height);
	// g_print ("++++++++++++++++ %d, %d \n", 
	//  frmsize.discrete.width,
	//  frmsize.discrete.height);
     	frmsize.index++;
       }
    
     if (frmsize.type != V4L2_FRMSIZE_TYPE_DISCRETE)
       {
	 description.frame_size_stepwise_max_width_ = frmsize.stepwise.max_width;
	 description.frame_size_stepwise_min_width_ = frmsize.stepwise.min_width;
	 description.frame_size_stepwise_step_width_ = frmsize.stepwise.step_width;
	 description.frame_size_stepwise_max_height_ = frmsize.stepwise.max_height;
	 description.frame_size_stepwise_min_height_ = frmsize.stepwise.min_height;
	 description.frame_size_stepwise_step_height_ = frmsize.stepwise.step_height;
	 default_width = frmsize.stepwise.max_width ;
	 default_height = frmsize.stepwise.max_height;
       }
     else
       {
	 description.frame_size_stepwise_max_width_ = -1;
	 description.frame_size_stepwise_min_width_ = -1;
	 description.frame_size_stepwise_step_width_ = -1;
	 description.frame_size_stepwise_max_height_ = -1;
	 description.frame_size_stepwise_min_height_ = -1;
	 description.frame_size_stepwise_step_height_ = -1;
       }
    
     v4l2_standard std;
     memset(&std, 0, sizeof(std));
     std.index = 0;
     description.tv_standards_.push_back ("none");
     while (ioctl(fd, VIDIOC_ENUMSTD, &std) >= 0)
       {
	 description.tv_standards_.push_back ((char *)std.name);
	 //g_print ("TV standard %s\n", (char *)std.name);
	 std.index++;
       }

     v4l2_frmivalenum frmival;
     memset(&frmival, 0, sizeof(frmival));
     frmival.pixel_format = default_pixel_format;
     frmival.width = default_width;
     frmival.height = default_height;
     frmival.index = 0;
     
     //g_print ("frame interval for default pixel format and default frame size:\n");
     while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) >= 0 && frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
       {	
	 if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
	   {
	     char *numerator = g_strdup_printf ("%u",frmival.discrete.numerator);
	     char *denominator = g_strdup_printf ("%u",frmival.discrete.denominator);
	     
	     description.frame_interval_discrete_.push_back (std::make_pair (numerator,denominator)); 
	     g_free (numerator);
	     g_free (denominator);
	     // g_print ("       %u/%u \n", 
	     // 	      frmival.discrete.numerator,
	     // 	      frmival.discrete.denominator);
	   }
	 // else
	 //   g_debug ("V4L2Src: frame size is not discret");
	 frmival.index++; 
       }
     
     if(frmival.type != V4L2_FRMIVAL_TYPE_DISCRETE)
       {
	 // g_print ("frametime (s) for rate min %u/%u\nrate max %u/%u\n time step %u/%u\n",
	 // 	  frmival.stepwise.min.numerator,
	 // 	  frmival.stepwise.min.denominator,
	 // 	  frmival.stepwise.max.numerator,
	 // 	  frmival.stepwise.max.denominator,
	 // 	  frmival.stepwise.step.numerator,
	 // 	  frmival.stepwise.step.denominator);

	 description.frame_interval_stepwise_max_numerator_ = frmival.stepwise.max.numerator;
	 description.frame_interval_stepwise_max_denominator_ = frmival.stepwise.max.denominator;
	 description.frame_interval_stepwise_min_numerator_ = frmival.stepwise.max.numerator;
	 description.frame_interval_stepwise_min_denominator_ = frmival.stepwise.max.denominator;
	 description.frame_interval_stepwise_step_numerator_ = frmival.stepwise.step.numerator;
	 description.frame_interval_stepwise_step_denominator_ = frmival.stepwise.step.denominator;
       }
     else
       {
	 description.frame_interval_stepwise_max_numerator_ = -1;
	 description.frame_interval_stepwise_max_denominator_ = -1;
	 description.frame_interval_stepwise_min_numerator_ = -1;
	 description.frame_interval_stepwise_min_denominator_ = -1;
	 description.frame_interval_stepwise_step_numerator_ = -1;
	 description.frame_interval_stepwise_step_denominator_ = -1;
       }
     close(fd);

     capture_devices_.push_back (description);
     return true;
}


  bool
  V4L2Src::check_folder_for_v4l2_devices (const char *dir_path)
  {
    GFile *inspected_dir = g_file_new_for_commandline_arg (dir_path);

    gboolean res;
    GError *error;
    GFileEnumerator *enumerator;
    GFileInfo *info;
    GFile *descend;
    char *absolute_path;
    
    error = NULL;
    enumerator =
      g_file_enumerate_children (inspected_dir, "*",
				 G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, NULL,
				 &error);
    if (! enumerator)
      return false;
    error = NULL;
    info = g_file_enumerator_next_file (enumerator, NULL, &error);
    while ((info) && (!error))
      {
	descend = g_file_get_child (inspected_dir, g_file_info_get_name (info));
	absolute_path = g_file_get_path (descend);
	
	if (g_str_has_prefix (absolute_path, "/dev/video") 
	    /*|| g_str_has_prefix (absolute_path, "/dev/radio")
	    || g_str_has_prefix (absolute_path, "/dev/vbi")
	    || g_str_has_prefix (absolute_path, "/dev/vtx")*/)
	  {
	    inspect_file_device (absolute_path);
	  }
		  
	g_object_unref (descend);
	error = NULL;
	info = g_file_enumerator_next_file (enumerator, NULL, &error);
      }

    if (error != NULL)
      g_debug ("error not NULL");
    
    error = NULL;
    res = g_file_enumerator_close (enumerator, NULL, &error);
    if (res != TRUE)
      g_debug ("V4L2Src: file enumerator not properly closed");
    if (error != NULL)
      g_debug ("V4L2Src: error not NULL");
    g_object_unref (inspected_dir);
   
    return true;
  }

  bool
  V4L2Src::inspect_frame_rate (const char */*file_path*/,
			       unsigned /*pixel_format*/,
			       unsigned /*width*/,
			       unsigned /*height*/)
  {
    //FIXME, framerate can change according to pixel_format and resolution
    g_debug ("  V4L2Src::inspect_frame_rate: TODO");
    return false;
  }


  bool
  V4L2Src::on_start ()
  {
    uninstall_property ("resolution");
    uninstall_property ("width");
    uninstall_property ("height");
    uninstall_property ("tv_standard");
    uninstall_property ("device");
    uninstall_property ("framerate");
    // install_property (G_OBJECT (v4l2src_),"brightness","brightness", "Brightness");
    // install_property (G_OBJECT (v4l2src_),"contrast","contrast", "Contrast");
    // install_property (G_OBJECT (v4l2src_),"saturation","saturation", "Saturation");
    // install_property (G_OBJECT (v4l2src_),"hue","hue", "Hue");

    return true;
  }
  
  bool
  V4L2Src::on_stop ()
  {
    clean_elements ();
    install_property_by_pspec (custom_props_->get_gobject (), 
				devices_enum_spec_, 
				"device",
				"Capture Device");
    update_device_specific_properties (device_);
    // uninstall_property ("brightness");
    // uninstall_property ("contrast");
    // uninstall_property ("saturation");
    // uninstall_property ("hue");

    return true;
  }

 
  gchar *
  V4L2Src::get_capture_devices_json (void *user_data)
  {
    V4L2Src *context = static_cast<V4L2Src *> (user_data);

    if (context->capture_devices_description_ != NULL)
      g_free (context->capture_devices_description_);
    JSONBuilder::ptr builder (new JSONBuilder ());
    builder->reset();
    builder->begin_object ();
    builder->set_member_name ("capture devices");
    builder->begin_array ();

    for (auto &it: context->capture_devices_)
      {
	builder->begin_object ();
	 builder->add_string_member ("long name", it.card_.c_str());
	 builder->add_string_member ("file path", it.file_device_.c_str());
	 builder->add_string_member ("bus info", it.bus_info_.c_str());

	 builder->set_member_name ("resolutions list");
	 builder->begin_array ();
	 for (auto &frame_size_it: it.frame_size_discrete_)
	   {
	     builder->begin_object ();
	     builder->add_string_member ("width", frame_size_it.first.c_str ());
	     builder->add_string_member ("height", frame_size_it.second.c_str ());
	     builder->end_object ();
	   }
	 builder->end_array ();
	 char *stepwise_max_width = g_strdup_printf ("%d",it.frame_size_stepwise_max_width_);
	 char *stepwise_min_width = g_strdup_printf ("%d",it.frame_size_stepwise_min_width_);
	 char *stepwise_step_width = g_strdup_printf ("%d",it.frame_size_stepwise_step_width_);
	 char *stepwise_max_height = g_strdup_printf ("%d",it.frame_size_stepwise_max_height_);
	 char *stepwise_min_height = g_strdup_printf ("%d",it.frame_size_stepwise_min_height_);
	 char *stepwise_step_height = g_strdup_printf ("%d",it.frame_size_stepwise_step_height_);
	 builder->add_string_member ("stepwise max width", stepwise_max_width);
	 builder->add_string_member ("stepwise min width", stepwise_min_width);
	 builder->add_string_member ("stepwise step width", stepwise_step_width);
	 builder->add_string_member ("stepwise max height", stepwise_max_height);
	 builder->add_string_member ("stepwise min height", stepwise_min_height );
	 builder->add_string_member ("stepwise step height", stepwise_step_height);
	 g_free (stepwise_max_width);
	 g_free (stepwise_min_width);
	 g_free (stepwise_step_width);
	 g_free (stepwise_max_height);
	 g_free (stepwise_min_height);
	 g_free (stepwise_step_height);
	 builder->set_member_name ("tv standards list");
	 builder->begin_array ();
	 for (auto& tv_standards_it: it.tv_standards_)
	     builder->add_string_value (tv_standards_it.c_str ());
	 builder->end_array ();

	 builder->set_member_name ("frame interval list (sec.)");
	 builder->begin_array ();
	 for (auto& frame_interval_it: it.frame_interval_discrete_)
	   {
	     builder->begin_object ();
	     builder->add_string_member ("numerator", frame_interval_it.first.c_str ());
	     builder->add_string_member ("denominator", frame_interval_it.second.c_str ());
	     builder->end_object ();
	   }
	 builder->end_array ();

	 char *stepwise_max_numerator = g_strdup_printf ("%d",it.frame_interval_stepwise_max_numerator_);
	 char *stepwise_min_numerator = g_strdup_printf ("%d",it.frame_interval_stepwise_min_numerator_);
	 char *stepwise_step_numerator = g_strdup_printf ("%d",it.frame_interval_stepwise_step_numerator_);
	 char *stepwise_max_denominator = g_strdup_printf ("%d",it.frame_interval_stepwise_max_denominator_);
	 char *stepwise_min_denominator = g_strdup_printf ("%d",it.frame_interval_stepwise_min_denominator_);
	 char *stepwise_step_denominator = g_strdup_printf ("%d",it.frame_interval_stepwise_step_denominator_);
	 
	 builder->add_string_member ("stepwise max frame interval numerator", 
				     stepwise_max_numerator);
	 builder->add_string_member ("stepwise max frame interval denominator", 
				     stepwise_max_denominator);
	 builder->add_string_member ("stepwise min frame interval numerator", 
				     stepwise_min_numerator);
	 builder->add_string_member ("stepwise min frame interval denominator", 
				     stepwise_min_denominator);
	 builder->add_string_member ("stepwise step frame interval numerator", 
				     stepwise_step_numerator);
	 builder->add_string_member ("stepwise step frame interval denominator", 
				     stepwise_step_denominator);
	 g_free (stepwise_max_numerator);
	 g_free (stepwise_min_numerator);
	 g_free (stepwise_step_numerator);
	 g_free (stepwise_max_denominator);
	 g_free (stepwise_min_denominator);
	 g_free (stepwise_step_denominator);
	builder->end_object ();
      }

    builder->end_array ();
    builder->end_object ();
    context->capture_devices_description_ = g_strdup (builder->get_string (true).c_str ());
    return context->capture_devices_description_;
  }


  void 
  V4L2Src::set_camera (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->device_ = value;
    context->update_device_specific_properties (context->device_);
  }
  
  gint 
  V4L2Src::get_camera (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->device_;
  }

  void 
  V4L2Src::set_resolution (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->resolution_ = value;
  }
  
  gint 
  V4L2Src::get_resolution (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->resolution_;
  }

  void 
  V4L2Src::set_width (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->width_ = value;
  }
  
  gint 
  V4L2Src::get_width (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->width_;
  }

  void 
  V4L2Src::set_height (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->height_ = value;
  }
  
  gint 
  V4L2Src::get_height (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->height_;
  }

  void 
  V4L2Src::set_tv_standard (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->tv_standard_ = value;
  }
  
  gint 
  V4L2Src::get_tv_standard (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->tv_standard_;
  }

  void 
  V4L2Src::set_framerate (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->framerate_ = value;
  }
  
  gint 
  V4L2Src::get_framerate (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->framerate_;
  }

  void 
  V4L2Src::set_framerate_numerator (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->framerate_numerator_ = value;
  }
  
  gint 
  V4L2Src::get_framerate_numerator (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->framerate_numerator_;
  }

  void 
  V4L2Src::set_framerate_denominator (const gint value, void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    context->framerate_denominator_ = value;
  }
  
  gint 
  V4L2Src::get_framerate_denominator (void *user_data)
  {
    V4L2Src *context = static_cast <V4L2Src *> (user_data);
    return context->framerate_denominator_;
  }

  bool 
  V4L2Src::make_video_source (GstElement **new_element)
  {
    if (capture_devices_.empty ())
      {
	g_debug ("V4L2Src:: no capture device available for starting capture");
	return false;
      }

    make_elements ();

    g_object_set (G_OBJECT (v4l2src_), 
		  "device", capture_devices_.at (device_).file_device_.c_str (), 
		  NULL);
    
    if (tv_standard_ > 0) //0 is none
      g_object_set (G_OBJECT (v4l2src_), 
		    "norm", capture_devices_.at (device_).tv_standards_.at (tv_standard_).c_str (), 
		    NULL);
    
    std::string caps;
    caps = "video/x-raw-yuv";
    if (width_ > 0)
      {
	gchar *width = g_strdup_printf ("%d",width_);
	gchar *height = g_strdup_printf ("%d",height_);
	caps = caps + ", width=(int)"+ width + ", height=(int)" + height;
	g_free (width);
	g_free (height);
      }
    else if (resolution_ > -1)
      {
	caps = 
	  caps + ", width=(int)" 
	  + capture_devices_.at (device_).frame_size_discrete_.at (resolution_).first.c_str () 
	  + ", height=(int)" 
	  + capture_devices_.at (device_).frame_size_discrete_.at (resolution_).second.c_str ();
      }

    if (framerate_ > 0)
      {
	caps = 
	  caps + ", framerate=(fraction)" 
	  + capture_devices_.at (device_).frame_interval_discrete_.at (framerate_).second.c_str () 
	  + "/" 
	  + capture_devices_.at (device_).frame_interval_discrete_.at (framerate_).first.c_str () ;
      }
    else if (framerate_numerator_ > -1)
      {
	gchar *numerator = g_strdup_printf ("%d",framerate_numerator_);
	gchar *denominator = g_strdup_printf ("%d",framerate_denominator_);
	caps = 
	  caps + ", framerate=(fraction)" 
	  + numerator 
	  + "/" 
	  + denominator ;
	g_free (numerator);
	g_free (denominator);
      }
    
    GstCaps *usercaps = gst_caps_from_string (caps.c_str ());
    g_object_set (G_OBJECT (capsfilter_), 
   		  "caps",
   		  usercaps,
   		  NULL);
    gst_caps_unref (usercaps);

    *new_element = v4l2_bin_;
    return true;
  }

}
