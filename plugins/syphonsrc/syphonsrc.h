/*
 * This file is part of switcher-top.
 *
 * switcher-top is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_SYPHONSRC_H__
#define __SWITCHER_SYPHONSRC_H__

#include "switcher/segment.h"
#include "switcher/startable-quiddity.h"
#include "switcher/custom-property-helper.h"

#include <memory>
#include <thread>

namespace switcher
{
  class SyphonSrc : public Segment
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(SyphonSrc);
    SyphonSrc ();
    ~SyphonSrc ();
    SyphonSrc (const SyphonSrc &) = delete;
    SyphonSrc &operator= (const SyphonSrc &) = delete;
    bool init_segment ();
    bool init ();

  private:
  };
  
  SWITCHER_DECLARE_PLUGIN(SyphonSrc);

}  // end of namespace

#endif // ifndef
