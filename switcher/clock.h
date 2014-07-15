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


#ifndef __SWITCHER_CLOCK_H__
#define __SWITCHER_CLOCK_H__

#include <chrono>

namespace switcher
{

  template <typename Clock_type = std::chrono::system_clock>
    class CumulativeClock
    {
    public:
      CumulativeClock () :
      start_ (Clock_type::now ())
	{}
      
    template <typename Count_type = unsigned long long, typename Resolution = std::nano>
    Count_type
    get_count ()
    {
      std::chrono::time_point<Clock_type> now = Clock_type::now();
      std::chrono::duration<Count_type, Resolution> clock = now - start_;
      return clock.count ();
    }
    
    private:
      std::chrono::time_point<Clock_type> start_;
    };

}  // end of namespace
#endif // ifndef
  
