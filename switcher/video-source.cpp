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

#include "video-source.h"
#include "gst-utils.h"

namespace switcher
{
  VideoSource::VideoSource () 
  {

    video_tee_ = NULL;
    // colorspace_in_= NULL;
    // textoverlay_ = NULL;
    // videoflip_ = NULL;
    // colorspace_out_ = NULL;
    videocaps_ = gst_caps_new_simple ("video/x-raw-yuv",
				      // "format", GST_TYPE_FOURCC,
				      // GST_MAKE_FOURCC ('U', 'Y', 'V', 'Y'),
				      // "format", GST_TYPE_FOURCC,
				      // GST_MAKE_FOURCC ('I', '4', '2', '0'),
				      //"framerate", GST_TYPE_FRACTION, 30, 1,
				      // "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, 
				      //  "width", G_TYPE_INT, 640, 
				      //  "height", G_TYPE_INT, 480,
				      NULL);
    
  }

  VideoSource::~VideoSource () 
  {
    clean_elements ();
    gst_caps_unref (videocaps_);
  }

  void 
  VideoSource::make_elements () //FIXME rename that
  {
    if (!GstUtils::make_element ("tee",&video_tee_))
      g_warning ("VideoSource: tee element is mandatory\n");
    // if (!GstUtils::make_element ("ffmpegcolorspace",&colorspace_in_))
    //   g_warning ("VideoSource: ffmpegcolorspace element is mandatory\n");
    // if (!GstUtils::make_element ("textoverlay",&textoverlay_))
    //   g_warning ("VideoSource: textoverlay element is mandatory\n");
    // if (!GstUtils::make_element ("videoflip",&videoflip_))
    //   g_warning ("VideoSource: videoflip element is mandatory\n");
    // GstUtils::make_element ("ffmpegcolorspace",&colorspace_out_);

    // cleaner_.reset (new GstElementCleaner ());
    // cleaner_->add_element_to_cleaner (video_tee_);
    // cleaner_->add_element_to_cleaner (colorspace_in_);
    // cleaner_->add_element_to_cleaner (textoverlay_);
    // cleaner_->add_element_to_cleaner (videoflip_);
    // cleaner_->add_element_to_cleaner (colorspace_out_);

     gst_bin_add_many (GST_BIN (bin_),
      		      video_tee_,
     		      //colorspace_in_,
      		      //textoverlay_,
      		      //videoflip_,
      		      //colorspace_out_,
      		      NULL);

    // gst_element_link_many (video_tee_,
    //  			   // colorspace_in_,
    //   			   // textoverlay_,
    //   			   // videoflip_,
    //   			   colorspace_out_,
    //   			   NULL);
    
  }

  void 
  VideoSource::clean_elements () //FIXME rename that
  {
    GstUtils::clean_element(video_tee_);
    unregister_shmdata_writer (make_file_name ("video"));
    // GstUtils::clean_element(colorspace_in_);
    // GstUtils::clean_element(textoverlay_);
    // GstUtils::clean_element(videoflip_);
    // GstUtils::clean_element(colorspace_out_);
  }

  void
  VideoSource::set_raw_video_element (GstElement *element)
  {
    reset_bin ();
    clean_elements ();
    make_elements ();

    rawvideo_ = element;
    
    gst_bin_add (GST_BIN (bin_), rawvideo_);
    gst_element_link (rawvideo_, video_tee_);
    
    //creating a connector for the raw video
    ShmdataWriter::ptr rawvideo_connector;
    rawvideo_connector.reset (new ShmdataWriter ());
    std::string rawconnector_name = make_file_name ("video"); 
    rawvideo_connector->set_path (rawconnector_name.c_str());
    rawvideo_connector->plug (bin_, video_tee_, videocaps_);
    register_shmdata_writer (rawvideo_connector);
    
    // //creating a connector for the transformed video
    // ShmdataWriter::ptr video_connector;
    // video_connector.reset (new ShmdataWriter ());
    // std::string connector_name = make_file_name ("transformed-video"); 
    // video_connector->set_path (connector_name.c_str());
    // video_connector->plug (bin_, colorspace_out_, videocaps_);
    // //video_connector->plug (bin_, video_tee_, videocaps);
    // register_shmdata_writer (video_connector);

    // g_debug ("VideoSource::set_raw_video_element (done)");

    GstUtils::wait_state_changed (bin_);
    GstUtils::sync_state_with_parent (rawvideo_);
    GstUtils::sync_state_with_parent (video_tee_);
    //GstUtils::sync_state_with_parent (colorspace_in_);
    //GstUtils::sync_state_with_parent (textoverlay_);
    //GstUtils::sync_state_with_parent (videoflip_);
    //GstUtils::sync_state_with_parent (colorspace_out_);

  }

}
