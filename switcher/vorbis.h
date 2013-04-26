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


#ifndef __SWITCHER_VORBIS_H__
#define __SWITCHER_VORBIS_H__

#include "audio-sink.h"
#include "gst-element-cleaner.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{
  class Vorbis : public AudioSink, public GstElementCleaner
  {
  public:
    typedef std::shared_ptr<Vorbis> ptr;
    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

    static void make_shmdata_writer(ShmdataReader *caller, void *vorbis_instance);

  private:
    GstElement *vorbisbin_;
    GstElement *vorbisenc_;
  };

}  // end of namespace

#endif // ifndef
