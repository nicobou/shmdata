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

  bool
  V4L2Src::init ()
  {

    if (!make_elements ())
      return false;

    //set the name before registering properties
    set_name (gst_element_get_name (v4l2src_));

    capture_devices_description_ = NULL;

    publish_method ("Capture",
		    "capture", 
		    "start capturing", 
		    "success or fail",
		    Method::make_arg_description ("Device",
						  "device_file_path",
						  "or default",
						  NULL),
		    (Method::method_ptr) &capture_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, 
						       NULL),
		    this);

    publish_method ("Capture Full",
		    "capture_full", 
		    "start capturing", 
		    "success or fail",
		    Method::make_arg_description ("Device",
						  "device_file_path",
						  "Device File Path or default",
						  "Width",
						  "width",
						  "or default",
						  "Height",
						  "height",
						  "or default",
						  "Framerate Numerator",
						  "framerate_numerator",
						  "or default",
						  "Framerate Denominator",
						  "framerate_denominator",
						  "or default",
						  "TV standard",
						  "tv_standard",
						  "or default",
						  NULL),
		    (Method::method_ptr) &capture_full_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, 
						       G_TYPE_STRING, 
						       G_TYPE_STRING, 
						       G_TYPE_STRING, 
						       G_TYPE_STRING,
						       G_TYPE_STRING, 
						       NULL),
		    this);

    //device inspector
    check_folder_for_v4l2_devices ("/dev");
    
    custom_props_.reset (new CustomPropertyHelper ());
    capture_devices_description_spec_ = custom_props_->make_string_property ("devices-json", 
									     "Description of capture devices (json formated)",
									     get_capture_devices_json (this),
									     (GParamFlags) G_PARAM_READABLE,
									     NULL,
									     V4L2Src::get_capture_devices_json,
									     this);
    

    register_property_by_pspec (custom_props_->get_gobject (), 
				capture_devices_description_spec_, 
				"devices-json",
				"Capture Devices",
				true,
				true);
    return true;
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

    //registering some properties FIXME unregister property and register property in make_element 
    unregister_property ("brightness");
    unregister_property ("contrast");
    unregister_property ("saturation");
    unregister_property ("hue");
    register_property (G_OBJECT (v4l2src_),"brightness","brightness", "Brightness", false, true);
    register_property (G_OBJECT (v4l2src_),"contrast","contrast", "Contrast", false, true);
    register_property (G_OBJECT (v4l2src_),"saturation","saturation", "Saturation", false, true);
    register_property (G_OBJECT (v4l2src_),"hue","hue", "Hue", false, true);

    
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
    //GstUtils::clean_element (v4l2src_);
    //GstUtils::clean_element (capsfilter_);//FIXME
    //GstUtils::clean_element (v4l2_bin_);
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
	 char *stepwise_max_width = g_strdup_printf ("%u",frmsize.stepwise.max_width);
	 char *stepwise_min_width = g_strdup_printf ("%u",frmsize.stepwise.min_width);
	 char *stepwise_step_width = g_strdup_printf ("%u",frmsize.stepwise.step_width);
	 char *stepwise_max_height = g_strdup_printf ("%u",frmsize.stepwise.max_height);
	 char *stepwise_min_height = g_strdup_printf ("%u",frmsize.stepwise.min_height);
	 char *stepwise_step_height = g_strdup_printf ("%u",frmsize.stepwise.step_height);
	 description.frame_size_stepwise_max_width_ = stepwise_max_width;
	 description.frame_size_stepwise_min_width_ = stepwise_min_width;
	 description.frame_size_stepwise_step_width_ = stepwise_step_width;
	 description.frame_size_stepwise_max_height_ = stepwise_max_height;
	 description.frame_size_stepwise_min_height_ = stepwise_min_height;
	 description.frame_size_stepwise_step_height_ = stepwise_step_height;
	 g_free (stepwise_max_width);
	 g_free (stepwise_min_width);
	 g_free (stepwise_step_width);
	 g_free (stepwise_max_height);
	 g_free (stepwise_min_height);
	 g_free (stepwise_step_height);
     	// g_print ("width %u %u (%u) --- height %u %u (%u)\n",
     	// 	 frmsize.stepwise.min_width,
     	// 	 frmsize.stepwise.max_width,
     	// 	 frmsize.stepwise.step_width,
     	// 	 frmsize.stepwise.min_height,
     	// 	 frmsize.stepwise.max_height,
     	// 	 frmsize.stepwise.step_height);
     	default_width = frmsize.stepwise.max_width ;
	default_height = frmsize.stepwise.max_height;
       }
    
     v4l2_standard std;
     memset(&std, 0, sizeof(std));
     std.index = 0;

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
	 char *stepwise_max_numerator = g_strdup_printf ("%u",frmival.stepwise.max.numerator);
	 char *stepwise_max_denominator = g_strdup_printf ("%u",frmival.stepwise.max.denominator);
	 description.frame_interval_stepwise_max_numerator_ = stepwise_max_numerator;
	 description.frame_interval_stepwise_max_denominator_ = stepwise_max_denominator;
	 g_free (stepwise_max_numerator);
	 g_free (stepwise_max_denominator);
	 char *stepwise_min_numerator = g_strdup_printf ("%u",frmival.stepwise.min.numerator);
	 char *stepwise_min_denominator = g_strdup_printf ("%u",frmival.stepwise.min.denominator);
	 description.frame_interval_stepwise_min_numerator_ = stepwise_min_numerator;
	 description.frame_interval_stepwise_min_denominator_ = stepwise_min_denominator;
	 g_free (stepwise_min_numerator);
	 g_free (stepwise_min_denominator);
	 char *stepwise_step_numerator = g_strdup_printf ("%u",frmival.stepwise.step.numerator);
	 char *stepwise_step_denominator = g_strdup_printf ("%u",frmival.stepwise.step.denominator);
	 description.frame_interval_stepwise_step_numerator_ = stepwise_step_numerator;
	 description.frame_interval_stepwise_step_denominator_ = stepwise_step_denominator;
	 g_free (stepwise_step_numerator);
	 g_free (stepwise_step_denominator);

       }
     close(fd);

     capture_devices_[file_path]= description;
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
	    //g_print ("Coucou ------------------------ %s\n", absolute_path);
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
  V4L2Src::inspect_frame_rate (const char *file_path,
			       unsigned pixel_format,
			       unsigned width,
			       unsigned height)
  {
    g_debug ("  V4L2Src::inspect_frame_rate: TODO");
    return false;
  }


  
  bool 
  V4L2Src::capture_full (const char *device_file_path, 
			 const char *width,
			 const char *height,
			 const char *framerate_numerator,
			 const char *framerate_denominator,
			 const char *tv_standard)
  {
    make_elements ();

    if (g_strcmp0 (device_file_path, "default") != 0)
      if (capture_devices_.find (device_file_path) != capture_devices_.end ())	
     	g_object_set (G_OBJECT (v4l2src_), "device", device_file_path, NULL);
      else
     	{
     	  g_warning ("V4L2Src: device %s is not a detected as a v4l2 device, cannot use", device_file_path);
     	  return false;
     	}
    
    if (g_strcmp0 (tv_standard, "default") != 0)
      g_object_set (G_OBJECT (v4l2src_), "norm", tv_standard, NULL);
    
    std::string caps;
    caps = "video/x-raw-yuv";
    if ((g_strcmp0 (width, "default") != 0)
     	&& (g_strcmp0 (height, "default") != 0))
      caps = caps + ", width=(int)"+ width + ", height=(int)" + height;
    
    if ((g_strcmp0 (framerate_numerator, "default") != 0)
     	&& (g_strcmp0 (framerate_denominator, "default") != 0))
      caps = caps + ", framerate=(fraction)" + framerate_numerator + "/" + framerate_denominator;
    
    //g_print ("v4l2 -- forcing caps %s", caps.c_str ());
    GstCaps *usercaps = gst_caps_from_string (caps.c_str ());
    g_object_set (G_OBJECT (capsfilter_), 
		  "caps",
		  usercaps,
		  NULL);
    gst_caps_unref (usercaps);

    set_raw_video_element (v4l2_bin_);
    
    return true;
  }
  
  gboolean 
  V4L2Src::capture_full_wrapped (gpointer device_file_path, 
				 gpointer width,
				 gpointer height,
				 gpointer framerate_numerator,
				 gpointer framerate_denominator, 
				 gpointer tv_standard,
				 gpointer user_data)
  {
    V4L2Src *context = static_cast<V4L2Src *>(user_data);
    
    if (context->capture_full ((const char *)device_file_path, 
			       (const char *)width,
			       (const char *)height,
			       (const char *)framerate_numerator,
			       (const char *)framerate_denominator,
			       (const char *)tv_standard))
      return TRUE;
    else
      return FALSE;
  }

  gboolean 
  V4L2Src::capture_wrapped (gpointer device_file_path, 
			    gpointer user_data)
  {
    V4L2Src *context = static_cast<V4L2Src *>(user_data);
    
    if (context->capture_full ((const char *)device_file_path, 
			       "default",
			       "default",
			       "default",
			       "default",
			       "default"))
      return TRUE;
    else
      return FALSE;
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
	builder->add_string_member ("long name", it.second.card_.c_str());
	builder->add_string_member ("file path", it.second.file_device_.c_str());
	builder->add_string_member ("bus info", it.second.bus_info_.c_str());

	builder->set_member_name ("resolutions list");
	builder->begin_array ();
	for (auto& frame_size_it: it.second.frame_size_discrete_)
	  {
	    builder->begin_object ();
	    builder->add_string_member ("width", frame_size_it.first.c_str ());
	    builder->add_string_member ("height", frame_size_it.second.c_str ());
	    builder->end_object ();
	  }
	builder->end_array ();

	builder->add_string_member ("stepwise max width", it.second.frame_size_stepwise_max_width_.c_str());
	builder->add_string_member ("stepwise min width", it.second.frame_size_stepwise_min_width_.c_str());
	builder->add_string_member ("stepwise step width", it.second.frame_size_stepwise_step_width_.c_str());
	builder->add_string_member ("stepwise max height", it.second.frame_size_stepwise_max_height_.c_str());
	builder->add_string_member ("stepwise min height", it.second.frame_size_stepwise_min_height_.c_str());
	builder->add_string_member ("stepwise step height", it.second.frame_size_stepwise_step_height_.c_str());

	builder->set_member_name ("tv standards list");
	builder->begin_array ();
	builder->add_string_value ("default");
	for (auto& tv_standards_it: it.second.tv_standards_)
	    builder->add_string_value (tv_standards_it.c_str ());
	builder->end_array ();

	builder->set_member_name ("frame interval list (sec.)");
	builder->begin_array ();
	for (auto& frame_interval_it: it.second.frame_interval_discrete_)
	  {
	    builder->begin_object ();
	    builder->add_string_member ("numerator", frame_interval_it.first.c_str ());
	    builder->add_string_member ("denominator", frame_interval_it.second.c_str ());
	    builder->end_object ();
	  }
	builder->end_array ();

	builder->add_string_member ("stepwise max numerator", 
				    it.second.frame_interval_stepwise_max_numerator_.c_str());
	builder->add_string_member ("stepwise max denominator", 
				    it.second.frame_interval_stepwise_max_denominator_.c_str());
	builder->add_string_member ("stepwise min numerator", 
				    it.second.frame_interval_stepwise_min_numerator_.c_str());
	builder->add_string_member ("stepwise min denominator", 
				    it.second.frame_interval_stepwise_min_denominator_.c_str());
	builder->add_string_member ("stepwise step numerator", 
				    it.second.frame_interval_stepwise_step_numerator_.c_str());
	builder->add_string_member ("stepwise step denominator", 
				    it.second.frame_interval_stepwise_step_denominator_.c_str());
	
	builder->end_object ();
      }

    builder->end_array ();
    builder->end_object ();
    context->capture_devices_description_ = g_strdup (builder->get_string (true).c_str ());
    return context->capture_devices_description_;
  }


}
