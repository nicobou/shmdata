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

#include "switcher/gst-element-cleaner.h"

namespace switcher
{

  GstElementCleaner::~GstElementCleaner ()
  {
      std::vector<GstElement *>::iterator element;
      for (element = elements_to_remove_.begin(); element != elements_to_remove_.end (); element ++)
	{
	  GstIterator *pad_iter;
	  pad_iter = gst_element_iterate_pads (*element);
	  gst_iterator_foreach (pad_iter, (GFunc) unlink_pad, *element);
	  gst_iterator_free (pad_iter);
	  gst_element_set_state (*element, GST_STATE_NULL);
	  gst_bin_remove (GST_BIN (gst_element_get_parent (*element)), *element);
	}
  }

  void
  GstElementCleaner::unlink_pad (GstPad * pad)
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
  GstElementCleaner::add_element_to_remove (GstElement *element)
  {
    elements_to_remove_.push_back (element);
  }

}
