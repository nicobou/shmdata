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
    textoverlay_ (gst_element_factory_make ("textoverlay",NULL)),
    videoflip_ (gst_element_factory_make ("videoflip",NULL)),
    deinterlace_ (gst_element_factory_make ("deinterlace",NULL)),
    identity_ (gst_element_factory_make ("identity",NULL)),
    xvimagesink_ (gst_element_factory_make ("xvimagesink",NULL))
  {
    g_print ("video source constructor \n");
    
    gst_bin_add_many (GST_BIN (bin_),
		      rawvideo_connector_->get_bin (),
		      textoverlay_,
		      videoflip_,
		      deinterlace_,
		      identity_,
		      xvimagesink_,
		      NULL);
    gst_element_link_many (textoverlay_,
			   videoflip_,
			   deinterlace_,
			   identity_,
			   xvimagesink_,
			   NULL);
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
