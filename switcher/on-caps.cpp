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

#include "on-caps.h"
#include "gst-utils.h"

namespace switcher
{

  
  std::string 
  OnCaps::get_caps () 
  {
    return negociated_caps_;
  }

  void 
  OnCaps::set_on_caps (CapsCallBack callback)
  {
    std::unique_lock<std::mutex> lock (caps_mutex_);
    on_caps_callback_.push_back (callback);
    if (!negociated_caps_.empty ())
      callback (negociated_caps_);
  }

  bool
  OnCaps::set_negociated_caps (std::string caps)
  {
    if (caps.empty ())
      return false;

    std::unique_lock<std::mutex> lock (caps_mutex_);
    negociated_caps_ = std::move (caps);
    for (auto &it : on_caps_callback_)
      it (negociated_caps_);
    return true;
  }
}
