/*
 * This file is part of posture.
 *
 * posture is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_POSTURE_DISPLAY_H__
#define __SWITCHER_POSTURE_DISPLAY_H__

#include <posture.h>

#include "switcher/quiddity.h"
#include "switcher/segment.h"
#include "switcher/custom-property-helper.h"

#include <memory>
#include <mutex>
#include <string>

namespace switcher
{
  class PostureDisplay : public Quiddity, public Segment
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureDisplay);
    PostureDisplay ();
    ~PostureDisplay ();
    PostureDisplay (const PostureDisplay &) = delete;
    PostureDisplay &operator= (const PostureDisplay &) = delete;

  private:
    CustomPropertyHelper::ptr custom_props_;

    std::shared_ptr<posture::Display> display_ {nullptr};

    bool init () final;
    
    bool connect (std::string shmdata_socket_path);
    bool disconnect_all ();
    bool can_sink_caps (std::string caps);

  };
  
  SWITCHER_DECLARE_PLUGIN(PostureDisplay);

}  // end of namespace

#endif // ifndef
