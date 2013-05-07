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

#include "video-source.h"
#include "gst-utils.h"

namespace switcher
{
  VideoSource::VideoSource () 
  {
    if (!GstUtils::make_element ("tee",&video_tee_))
      g_warning ("VideoSource: tee element is mandatory\n");
    if (!GstUtils::make_element ("ffmpegcolorspace",&colorspace_in_))
      g_warning ("VideoSource: ffmpegcolorspace element is mandatory\n");
    if (!GstUtils::make_element ("textoverlay",&textoverlay_))
      g_warning ("VideoSource: textoverlay element is mandatory\n");
    if (!GstUtils::make_element ("videoflip",&videoflip_))
      g_warning ("VideoSource: videoflip element is mandatory\n");
    GstUtils::make_element ("ffmpegcolorspace",&colorspace_out_);

    cleaner_.reset (new GstElementCleaner ());
    cleaner_->add_element_to_cleaner (video_tee_);
    cleaner_->add_element_to_cleaner (colorspace_in_);
    cleaner_->add_element_to_cleaner (textoverlay_);
    cleaner_->add_element_to_cleaner (videoflip_);
    cleaner_->add_element_to_cleaner (colorspace_out_);

    gst_bin_add_many (GST_BIN (bin_),
     		      video_tee_,
		      colorspace_in_,
     		      textoverlay_,
     		      videoflip_,
     		      colorspace_out_,
     		      NULL);
    
    gst_element_link_many (video_tee_,
			   colorspace_in_,
     			   textoverlay_,
     			   videoflip_,
     			   colorspace_out_,
     			   NULL);
    
    //registering selected properties
    register_property (G_OBJECT (videoflip_),"method","flip-method");
    register_property (G_OBJECT (textoverlay_),"text","text");
    register_property (G_OBJECT (textoverlay_),"auto-resize","auto-resize-text");
    // register_property (G_OBJECT (textoverlay_),"shaded-background","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"halignment","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"valignment","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"xpad","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"ypad","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"deltax","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"deltay","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"xpos","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"ypos","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"wrap-mode","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"font-desc","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"silent","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"line-alignment","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"wait-text","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"vertical-render","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"color","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"shadow","textoverlay");
    // register_property (G_OBJECT (textoverlay_),"outline-color","textoverlay");
  }

  void
  VideoSource::set_raw_video_element (GstElement *element)
  {

    rawvideo_ = element;
    
    GstCaps *videocaps = gst_caps_new_simple ("video/x-raw-yuv",
					      // "format", GST_TYPE_FOURCC,
					      // GST_MAKE_FOURCC ('U', 'Y', 'V', 'Y'),
					      //"format", GST_TYPE_FOURCC,
					      //  GST_MAKE_FOURCC ('I', '4', '2', '0'),
					      //"framerate", GST_TYPE_FRACTION, 30, 1,
					      // "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, 
					      //  "width", G_TYPE_INT, 640, 
					      //  "height", G_TYPE_INT, 480,
					      NULL);
    
    gst_bin_add (GST_BIN (bin_), rawvideo_);
    gst_element_link (rawvideo_, video_tee_);
    
    //creating a connector for the raw video
    ShmdataWriter::ptr rawvideo_connector;
    rawvideo_connector.reset (new ShmdataWriter ());
    std::string rawconnector_name = make_file_name ("rawvideo"); 
    rawvideo_connector->set_path (rawconnector_name.c_str());
    rawvideo_connector->plug (bin_, video_tee_, videocaps);
    register_shmdata_writer (rawvideo_connector);
    
    //creating a connector for the transformed video
    ShmdataWriter::ptr video_connector;
    video_connector.reset (new ShmdataWriter ());
    std::string connector_name = make_file_name ("video"); 
    video_connector->set_path (connector_name.c_str());
    video_connector->plug (bin_, colorspace_out_, videocaps);
    register_shmdata_writer (video_connector);

    g_debug ("VideoSource::set_raw_video_element (done)");

    //gst_object_unref (videocaps);
  }

}
