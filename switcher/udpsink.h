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


#ifndef __SWITCHER_UDPSINK_H__
#define __SWITCHER_UDPSINK_H__

#include "switcher/base-sink.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class UDPSink : public BaseSink
  {
  public:
    typedef std::tr1::shared_ptr<UDPSink> ptr;
    UDPSink ();
    UDPSink (QuiddityLifeManager::ptr life_manager);

    static QuiddityDocumentation get_documentation ();

  private:
    static QuiddityDocumentation doc_;
    GstElement *udpsink_;
    void make_udpsink ();
  };

}  // end of namespace

#endif // ifndef
