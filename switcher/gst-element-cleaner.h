/*
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


#ifndef __SWITCHER_GST_ELEMENT_CLEANER_H__
#define __SWITCHER_GST_ELEMENT_CLEANER_H__

#include <gst/gst.h>
#include <memory>
#include <vector>
#include <map>
#include <string>

namespace switcher
{
  class GstElementCleaner
  {
  public:
    typedef std::shared_ptr<GstElementCleaner> ptr;
    GstElementCleaner ();
    virtual ~GstElementCleaner ();
    void add_element_to_cleaner (GstElement *element);
    void add_labeled_element_to_cleaner (const std::string &new_label, GstElement *element);
    GstElement *get_labeled_element_from_cleaner (const std::string &new_label);

  private:
    std::vector<GstElement *> elements_to_remove_;
    std::map<std::string, GstElement *> labeled_elements_;
  };
}  // end of namespace

#endif // ifndef
