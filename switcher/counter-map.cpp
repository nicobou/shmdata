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

#include "counter-map.h"

namespace switcher
{

  CounterMap::CounterMap () :
    counters_ (),
    mutex_ ()
  {}

  CounterMap::~CounterMap () {}

  uint
  CounterMap::get_count (const std::string &key)
  {
    std::unique_lock<std::mutex> lock (mutex_);
    auto it = counters_.find (key);
    if (counters_.end () != it)
      return ++(it->second);
    //else init to 0 for this key
    counters_[key] = 0;
    return 0;
  }

  void 
  CounterMap::reset_counter_map ()
  {
    std::unique_lock<std::mutex> lock (mutex_);
    counters_.clear ();
  }
}//end of switcher namespace

