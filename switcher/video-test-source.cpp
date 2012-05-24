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

#include "switcher/video-test-source.h"

namespace switcher
{

  VideoTestSource::VideoTestSource ()
  {
    g_print ("video test source constructor \n");
    videotestsrc_ = gst_element_factory_make ("videotestsrc",NULL);
    name_ = gst_element_get_name (videotestsrc_);
    set_raw_video_element (videotestsrc_);
  }

}
