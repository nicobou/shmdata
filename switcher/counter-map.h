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


#ifndef __SWITCHER_COUNTER_MAP_H__
#define __SWITCHER_COUNTER_MAP_H__

#include <map>
#include <mutex>

/**
 *
 * maintains a thread safe map of 
 * string as key and a uint counter as value
 *
 * users should inherit from this class
 *
**/  

namespace switcher
{

  class CounterMap
  {
  public:
    CounterMap ();
    virtual ~CounterMap ();

  protected:
    uint get_count (const std::string &key);
    void reset_counter_map ();

  private:
    std::map<std::string, uint> counters_;
    std::mutex mutex_;
 };

}  // end of namespace

#endif // ifndef
