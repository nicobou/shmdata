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

#include "switcher/gst-utils.h"

namespace switcher
{

  bool 
  GstUtils::link_static_to_request (GstElement *src,GstElement *sink)
  {
    GstPad *srcpad =  gst_element_get_static_pad (src, "src");
    GstPad *sinkpad = gst_element_get_compatible_pad(sink,
						     srcpad,
						     NULL); //const GstCaps *caps to use as a filter
    bool res = GstUtils::check_pad_link_return(gst_pad_link (srcpad,sinkpad));
    if (GST_IS_PAD (src))
	gst_object_unref (srcpad);

    if (GST_IS_PAD (sinkpad))
      gst_object_unref (sinkpad);

    return res;
  }

  bool 
  GstUtils::link_static_to_request (GstPad *srcpad,GstElement *sink)
  {
    GstPad *sinkpad = gst_element_get_compatible_pad(sink,
						     srcpad,
						     NULL); //const GstCaps *caps to use as a filter
    bool res = GstUtils::check_pad_link_return(gst_pad_link (srcpad,sinkpad));
    
    if (GST_IS_PAD (sinkpad))
      gst_object_unref (sinkpad);

    return res;
  }

  bool 
  GstUtils::check_pad_link_return (GstPadLinkReturn res)
  {
    if (res == GST_PAD_LINK_OK)
      return true;
    else
      {
	switch ( res )
	  {
	  case GST_PAD_LINK_WRONG_HIERARCHY:
	    g_error ("GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_HIERARCHY");
	    break;
	  case GST_PAD_LINK_WAS_LINKED:
	    g_error ("GstUtils::check_pad_link_return - GST_PAD_LINK_WAS_LINKED");
	    break;
	  case GST_PAD_LINK_WRONG_DIRECTION:
	    g_error ("GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_DIRECTION");
	    break;
	  case GST_PAD_LINK_NOFORMAT:
	    g_error ("GstUtils::check_pad_link_return - GST_PAD_LINK_NOFORMAT");
	    break;
	  case GST_PAD_LINK_NOSCHED:
	    g_error ("GstUtils::check_pad_link_return - GST_PAD_LINK_NOSCHED");
	    break;
	  case GST_PAD_LINK_REFUSED:
	    g_error ("GstUtils::check_pad_link_return - GST_PAD_LINK_REFUSED");
	    break;
	  default:
	    g_error ("GstUtils::check_pad_link_return - UNKNOWN ERROR");
	  }
	return false;
      }
  }
  
}
