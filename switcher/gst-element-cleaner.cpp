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

#include "gst-element-cleaner.h"
#include "gst-utils.h"

namespace switcher
{

  GstElementCleaner::GstElementCleaner () :
    elements_to_remove_ (),
    labeled_elements_ ()
  {}

  GstElementCleaner::~GstElementCleaner ()
  {
    for (auto &it : elements_to_remove_)
      GstUtils::clean_element (it);
  }

  void 
  GstElementCleaner::add_element_to_cleaner (GstElement *element)
  {
    elements_to_remove_.push_back (element);
  }

  void 
  GstElementCleaner::add_labeled_element_to_cleaner (const std::string &new_label, GstElement *element)
  {
    labeled_elements_[new_label] = element;
  }
  
  GstElement *
  GstElementCleaner::get_labeled_element_from_cleaner (const std::string &new_label)
  {
    auto it = labeled_elements_.find (new_label);
    if (labeled_elements_.end () == it)
      return nullptr;
    return it->second;
  }

}
