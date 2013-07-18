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


#ifndef __SWITCHER_V4L2SRC_H__
#define __SWITCHER_V4L2SRC_H__

#include "switcher/video-source.h"
#include "switcher/custom-property-helper.h"
#include <memory>

namespace switcher
{
  
  class V4L2Src : public VideoSource 
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(V4L2Src);
  
    bool capture (gboolean capture);
    
  private:
    GstElement *v4l2src_;
    static gboolean capture_wrapped (gboolean capture, gpointer user_data);
  };
  
  
  SWITCHER_DECLARE_PLUGIN(V4L2Src);

}  // end of namespace

#endif // ifndef
