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


#ifndef __SWITCHER_GST_ELEMENT_CLEANER_H__
#define __SWITCHER_GST_ELEMENT_CLEANER_H__

#include <vector>
#include <gst/gst.h>
#include <memory>
#include "string-map.h"

namespace switcher
{
  
  class GstElementCleaner
  {
  public:
    typedef std::shared_ptr<GstElementCleaner> ptr;
    ~GstElementCleaner ();
    void add_element_to_cleaner (GstElement *element);
    void add_labeled_element_to_cleaner (std::string label, GstElement *element);
    GstElement *get_labeled_element_from_cleaner (std::string label);

  private:
    std::vector<GstElement *> elements_to_remove_;
    StringMap<GstElement *> labeled_elements_;
  };
  
}  // end of namespace

#endif // ifndef
