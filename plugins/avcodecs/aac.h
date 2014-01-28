/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-avcodecs.
 *
 * switcher-avcodecs is free software: you can redistribute it and/or modify
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


#ifndef __SWITCHER_AAC_H__
#define __SWITCHER_AAC_H__

#include "switcher/audio-sink.h"
#include "switcher/gst-element-cleaner.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{
  class AAC : public AudioSink, public GstElementCleaner
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(AAC);
    AAC ();
    AAC (const AAC &) = delete;
    AAC &operator= (const AAC &) = delete;

    static void make_shmdata_writer(ShmdataReader *caller, void *aac_instance);

  private:
    GstElement *aacbin_;
    GstElement *aacenc_;
    bool init_segment ();
  };

  SWITCHER_DECLARE_PLUGIN(AAC);
}  // end of namespace

#endif // ifndef
