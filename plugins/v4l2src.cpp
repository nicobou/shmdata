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

#include "v4l2src.h"
#include "switcher/gst-utils.h"
#include <cstdlib>  // For srand() and rand()
#include <ctime>    // For time()

#include <linux/videodev2.h>
// #include <libv4l2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(V4L2Src,
				       "Video Capture (with v4l2)",
				       "video capture source", 
				       "Discover and use v4l2 supported capture cards and cameras",
				       "GPL",
				       "v4l2src",				
				       "Nicolas Bouillot");

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
    g_print ("\n----------- inspecting %s\n", file_path);
    struct v4l2_capability vcap;
    int fd = open(file_path, O_RDWR);
    ioctl(fd, VIDIOC_QUERYCAP, &vcap);
    g_print ("-------------------------- card %s bus %s driver %s\n", 
	     (char *)vcap.card,
	     (char *)vcap.bus_info,
	     (char *)vcap.driver);

    //pixel format
    v4l2_fmtdesc fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) >= 0)
      {
	g_print ("******** pixel format  %s - %s\n", 
		 pixel_format_to_string(fmt.pixelformat).c_str (), 
		 (const char *)fmt.description);
	fmt.index ++;
      }

    //void void GeneralTab::updateFrameSize()
    v4l2_frmsizeenum frmsize;
    memset(&frmsize, 0, sizeof(frmsize));
    frmsize.pixel_format = fmt.pixelformat;
    frmsize.index = 0;

    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0 && frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
      {
	g_print ("++++++++++++++++ %d, %d \n", 
		 frmsize.discrete.width,
		 frmsize.discrete.height);
	frmsize.index++;
      }
    
    if (frmsize.type != V4L2_FRMSIZE_TYPE_DISCRETE)
      {
	g_print ("width %u %u (%u) --- height %u %u (%u)\n",
		 frmsize.stepwise.min_width,
		 frmsize.stepwise.max_width,
		 frmsize.stepwise.step_width,
		 frmsize.stepwise.min_height,
		 frmsize.stepwise.max_height,
		 frmsize.stepwise.step_height);
      }
    
    v4l2_standard std;
    memset(&std, 0, sizeof(std));
    std.index = 0;

    while (ioctl(fd, VIDIOC_ENUMSTD, &std) >= 0)
      {
	g_print ("TV standard %s\n", (char *)std.name);
	std.index++;
      }
    close(fd);
    
  }

  bool
  V4L2Src::init ()
  {

    if (!GstUtils::make_element ("v4l2src",&v4l2src_))
      return false;
    
    //set the name before registering properties
    set_name (gst_element_get_name (v4l2src_));

    //registering "pattern"
    register_property (G_OBJECT (v4l2src_),"device","device", "Path To The Device");
    register_property (G_OBJECT (v4l2src_),"brightness","brightness", "Brightness");
    register_property (G_OBJECT (v4l2src_),"contrast","contrast", "Contrast");
    register_property (G_OBJECT (v4l2src_),"saturation","saturation", "Saturation");
    register_property (G_OBJECT (v4l2src_),"hue","hue", "Hue");
    register_property (G_OBJECT (v4l2src_),"norm","norm", "Video Standard");

    //registering capture
    register_method("capture",
		    (void *)&capture_wrapped, 
		    Method::make_arg_type_description (G_TYPE_BOOLEAN, NULL),
		    (gpointer)this);
    set_method_description ("capture", 
			    "start or stop capture", 
			    Method::make_arg_description ("capturing", 
							  "true for starting the capture, false for stoping",
							  NULL));

    inspect_file_device ("/dev/video0");
    inspect_file_device ("/dev/video1");
    inspect_file_device ("/dev/video2");
    inspect_file_device ("/dev/video3");
    inspect_file_device ("/dev/video63");
    
    return true;
  }

  
  bool 
  V4L2Src::capture (gboolean capture)
  {
    if (!capture)
      return false;
    g_print ("coucou2\n");
    
    set_raw_video_element (v4l2src_);
   
    return true;
  }

  gboolean 
  V4L2Src::capture_wrapped (gboolean capture, gpointer user_data)
  {
    V4L2Src *context = static_cast<V4L2Src *>(user_data);

    if (context->capture (capture))
      return TRUE;
    else
      return FALSE;
    
  }

}
