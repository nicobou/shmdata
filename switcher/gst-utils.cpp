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
    if (res)
	gst_object_unref (srcpad);

    return res;
  }

  bool 
  GstUtils::check_pad_link_return (GstPadLinkReturn res)
  {
    if (res == GST_PAD_LINK_OK)
      return true;
    else
      {
	g_printerr ("GstUtils::check_pad_link_return - ERROR %d (GstPadLinkReturn)\n", res);
	 // switch ( res )
	 //   {
	 //   case 'GST_PAD_LINK_WRONG_HIERARCHY':
         //     g_printerr ("GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_HIERARCHY\n");
         //     break;
	 //   case 'GST_PAD_LINK_WAS_LINKED':
         //     g_printerr ("GstUtils::check_pad_link_return - GST_PAD_LINK_WAS_LINKED\n");
         //     break;
	 //   case 'GST_PAD_LINK_WRONG_DIRECTION':
         //     g_printerr ("GstUtils::check_pad_link_return - GST_PAD_LINK_WRONG_DIRECTION\n");
         //     break;
	 //   case 'GST_PAD_LINK_NOFORMAT':
         //     g_printerr ("GstUtils::check_pad_link_return - GST_PAD_LINK_NOFORMAT\n");
         //     break;
	 //   case 'GST_PAD_LINK_NOSCHED':
         //     g_printerr ("GstUtils::check_pad_link_return - GST_PAD_LINK_NOSCHED\n");
         //     break;
	 //   case 'GST_PAD_LINK_REFUSED':
         //     g_printerr ("GstUtils::check_pad_link_return - GST_PAD_LINK_REFUSED\n");
         //     break;
	 //   default:
         //     g_printerr ("GstUtils::check_pad_link_return - UNKNOWN ERROR\n");
	 //   }
	return false;
      }
  }
    
}
