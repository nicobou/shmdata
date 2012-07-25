/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/video-source.h"

namespace switcher
{
  VideoSource::VideoSource () :
    rawvideo_connector_ (new Connector()),
    alpha_ (gst_element_factory_make ("alpha",NULL)),
    textoverlay_ (gst_element_factory_make ("textoverlay",NULL)),
    videoflip_ (gst_element_factory_make ("videoflip",NULL)),
    deinterlace_ (gst_element_factory_make ("deinterlace",NULL)),
    identity_ (gst_element_factory_make ("identity",NULL)),
    xvimagesink_ (gst_element_factory_make ("xvimagesink",NULL))
  {
    
    GstElement *colorspace = gst_element_factory_make ("ffmpegcolorspace",NULL);
    
    gst_bin_add_many (GST_BIN (bin_),
		      rawvideo_connector_->get_bin (),
		      textoverlay_,
		      videoflip_,
		      deinterlace_,
		      alpha_,
		      identity_,
		      colorspace,
		      xvimagesink_,
		      NULL);
    
    gst_element_link_many (textoverlay_,
			   videoflip_,
			   deinterlace_,
			   alpha_,
			   identity_,
			   colorspace,
			   xvimagesink_,
			   NULL);
    //properties
    register_property (G_OBJECT (videoflip_),"method","flip");
    
    register_property (G_OBJECT (deinterlace_),"mode","deinterlace");//default "Auto detection"
    register_property (G_OBJECT (deinterlace_),"method","deinterlace");

    register_property (G_OBJECT (alpha_),"method","alpha");
    register_property (G_OBJECT (alpha_),"alpha","alpha");
    register_property (G_OBJECT (alpha_),"target-r","alpha");
    register_property (G_OBJECT (alpha_),"target-g","alpha");
    register_property (G_OBJECT (alpha_),"target-b","alpha");
    register_property (G_OBJECT (alpha_),"angle","alpha");
    register_property (G_OBJECT (alpha_),"noise-level","alpha");
    register_property (G_OBJECT (alpha_),"black-sensitivity","alpha");
    register_property (G_OBJECT (alpha_),"white-sensitivity","alpha");
    register_property (G_OBJECT (alpha_),"prefer-passthrough","alpha");

    //properties for textoverlay
    register_property (G_OBJECT (textoverlay_),"text","textoverlay");
    register_property (G_OBJECT (textoverlay_),"shaded-background","textoverlay");
    register_property (G_OBJECT (textoverlay_),"halignment","textoverlay");
    register_property (G_OBJECT (textoverlay_),"valignment","textoverlay");
    register_property (G_OBJECT (textoverlay_),"xpad","textoverlay");
    register_property (G_OBJECT (textoverlay_),"ypad","textoverlay");
    register_property (G_OBJECT (textoverlay_),"deltax","textoverlay");
    register_property (G_OBJECT (textoverlay_),"deltay","textoverlay");
    register_property (G_OBJECT (textoverlay_),"xpos","textoverlay");
    register_property (G_OBJECT (textoverlay_),"ypos","textoverlay");
    register_property (G_OBJECT (textoverlay_),"wrap-mode","textoverlay");
    register_property (G_OBJECT (textoverlay_),"font-desc","textoverlay");
    register_property (G_OBJECT (textoverlay_),"silent","textoverlay");
    register_property (G_OBJECT (textoverlay_),"line-alignment","textoverlay");
    register_property (G_OBJECT (textoverlay_),"wait-text","textoverlay");
    register_property (G_OBJECT (textoverlay_),"auto-resize","textoverlay");
    register_property (G_OBJECT (textoverlay_),"vertical-render","textoverlay");
    register_property (G_OBJECT (textoverlay_),"color","textoverlay");
    register_property (G_OBJECT (textoverlay_),"shadow","textoverlay");
    register_property (G_OBJECT (textoverlay_),"outline-color","textoverlay");


  }

  void
  VideoSource::set_raw_video_element (GstElement *element)
  {
    rawvideo_ = element;
    gst_bin_add (GST_BIN (bin_),rawvideo_);
    gst_element_link (rawvideo_,rawvideo_connector_->get_sink_element());
    gst_element_link (rawvideo_connector_->get_src_element (),
		      textoverlay_);

  }

}
