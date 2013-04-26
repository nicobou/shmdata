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

#include "gst-element-cleaner.h"
#include "gst-utils.h"

namespace switcher
{

  GstElementCleaner::~GstElementCleaner ()
  {
    g_debug ("~GstElementCleaner");
      std::vector<GstElement *>::iterator element;
      for (element = elements_to_remove_.begin(); element != elements_to_remove_.end (); element ++)
	{
	  // GstIterator *pad_iter;
	  // pad_iter = gst_element_iterate_pads (*element);
	  // gst_iterator_foreach (pad_iter, (GFunc) GstUtils::unlink_pad, *element);
	  // gst_iterator_free (pad_iter);
	  // gst_element_set_state (*element, GST_STATE_NULL);
	  // if (GST_IS_BIN (gst_element_get_parent (*element)))
	  //   gst_bin_remove (GST_BIN (gst_element_get_parent (*element)), *element);
	  GstUtils::clean_element (*element);
	}
  }

  void 
  GstElementCleaner::add_element_to_cleaner (GstElement *element)
  {
    elements_to_remove_.push_back (element);
  }

    void 
    GstElementCleaner::add_labeled_element_to_cleaner (std::string label, GstElement *element)
    {
      labeled_elements_.insert (label, element);
    }
  
    GstElement *
    GstElementCleaner::get_labeled_element_from_cleaner (std::string label)
    {
      return labeled_elements_.lookup (label);
    }

}
