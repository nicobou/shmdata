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

  void
  GstUtils::unlink_pad (GstPad * pad)
  {
    GstPad *peer;
    if ((peer = gst_pad_get_peer (pad))) {
      if (gst_pad_get_direction (pad) == GST_PAD_SRC)
	gst_pad_unlink (pad, peer);
      else
	gst_pad_unlink (peer, pad);
      //checking if the pad has been requested and releasing it needed 
      GstPadTemplate *pad_templ = gst_pad_get_pad_template (peer);//check if this must be unrefed for GST 1
      if (GST_PAD_TEMPLATE_PRESENCE (pad_templ) == GST_PAD_REQUEST)
	gst_element_release_request_pad (gst_pad_get_parent_element(peer), peer);
      gst_object_unref (peer);
    }
  }

  void
  GstUtils::clean_element (GstElement *element)
  {
    if (element != NULL && GST_IS_ELEMENT (element))
      {
	if (GST_IS_BIN (gst_element_get_parent (element)))
	  GstUtils::wait_state_changed (GST_ELEMENT (gst_element_get_parent (element)));
	
	GstIterator *pad_iter;
	pad_iter = gst_element_iterate_pads (element);
	gst_iterator_foreach (pad_iter, (GFunc) GstUtils::unlink_pad, element);
	gst_iterator_free (pad_iter);
	if (GST_STATE_TARGET (element) != GST_STATE_NULL)
	  if (GST_STATE_CHANGE_ASYNC == gst_element_set_state (element, GST_STATE_NULL))
	    gst_element_get_state (element, NULL, NULL, GST_CLOCK_TIME_NONE);//warning this may be blocking
	if (GST_IS_BIN (gst_element_get_parent (element)))
	  gst_bin_remove (GST_BIN (gst_element_get_parent (element)), element);
      }
  }
  

  void
  GstUtils::wait_state_changed (GstElement *bin)
  {
    g_debug ("GstUtils::wait_state_changed");
    if (!GST_IS_BIN (bin))
      {
	g_warning ("GstUtils::wait_state_changed not a bin");
	return;
      }
    GValue val = G_VALUE_INIT;
    g_value_init (&val, G_TYPE_BOOLEAN);
    
    g_object_get_property (G_OBJECT (bin),
			   "async-handling",
			   &val);

    if (g_value_get_boolean (&val) == FALSE)
      if (GST_STATE (bin) != GST_STATE_TARGET (bin))
	gst_element_get_state (bin, NULL, NULL, GST_CLOCK_TIME_NONE);//warning this may be blocking
    
    g_debug ("GstUtils::wait_state_changed (done)");

    return;
  }

  void
  GstUtils::sync_state_with_parent (GstElement *element)
  {
    if (!GST_IS_ELEMENT (element))
      {
	g_debug ("GstUtils::sync_state_with_parent, arg is not an element");
	return;
      }
    
    GstElement *parent = GST_ELEMENT (gst_element_get_parent (element));
    if (GST_IS_ELEMENT (parent))
      {
	if (GST_STATE (parent) != GST_STATE_TARGET (parent))
	  gst_element_set_state (element, GST_STATE_TARGET (parent));
	else
	  gst_element_sync_state_with_parent (parent);
      }
    else
      g_warning ("GstUtils::sync_state_with_parent, cannot sync an orphan element");
  }
}
