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


#ifndef __SWITCHER_ARAVIS_GENICAM_H__
#define __SWITCHER_ARAVIS_GENICAM_H__

#include "video-source.h"
#include "aravis-genicam.h"
#include "gst-element-cleaner.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class AravisGenicam : public BaseSource, public GstElementCleaner
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(AravisGenicam);
    AravisGenicam ();
    AravisGenicam (const AravisGenicam &) = delete;
    AravisGenicam &operator= (const AravisGenicam &) = delete;

    bool start (std::string name);
  private:
    GstElement *aravissrc_;
    bool init_segment ();
    static gboolean start_wrapped (gpointer name, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
