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


#ifndef __SWITCHER_ON_CAPS_H__
#define __SWITCHER_ON_CAPS_H__

#include <string>
#include <list>
#include <mutex>

namespace switcher
{

  class OnCaps
  {
  public:
    using CapsCallBack = std::function<void(std::string)>;
    
    virtual ~OnCaps () {};

    void set_on_caps (CapsCallBack callback);
    std::string get_caps ();

  private:
    std::string negociated_caps_ {};
    std::list <CapsCallBack> on_caps_callback_ {};
    std::mutex caps_mutex_ {};

  protected:
    bool set_negociated_caps (std::string caps);
  };
  
}  // end of namespace

#endif // ifndef
