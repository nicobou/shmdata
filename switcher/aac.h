/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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


#ifndef __SWITCHER_AAC_H__
#define __SWITCHER_AAC_H__

#include "switcher/video-sink.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class AAC : public VideoSink
  {
  public:
    typedef std::shared_ptr<AAC> ptr;
    AAC ();
    AAC (QuiddityLifeManager::ptr life_manager);

    static void make_shmdata_writer(ShmdataReader *caller, void *aac_instance);

    QuiddityDocumentation get_documentation ();
    static const QuiddityDocumentation doc_;

  private:
    void make_aac ();
    GstElement *aacbin_;
    GstElement *aacenc_;
  };

}  // end of namespace

#endif // ifndef
