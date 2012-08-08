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


#ifndef __SWITCHER_CONNECTOR_H__
#define __SWITCHER_CONNECTOR_H__

#include <gst/gst.h>
#include <tr1/memory>

namespace switcher
{

  class Connector 
  {
  public:
    typedef std::tr1::shared_ptr<Connector> ptr;
    Connector ();
    GstElement *get_bin ();
    GstElement *get_src_element ();
    GstElement *get_sink_element ();
  private:
    GstElement *bin_;
    GstElement *tee_;
    GstElement *queue_;
  };

}  // end of namespace

#endif // ifndef
