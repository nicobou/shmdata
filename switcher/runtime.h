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

/**
 * The Runtime class
 */

#ifndef __SWITCHER_RUNTIME_H__
#define __SWITCHER_RUNTIME_H__

#include <gst/gst.h>
#include "switcher/quiddity.h"
#include <memory>

namespace switcher
{
  
  class Runtime : public Quiddity 
    {
    public:
      typedef std::shared_ptr<Runtime> ptr;
      ~Runtime ();
      bool init ();
      QuiddityDocumentation get_documentation ();
      static const QuiddityDocumentation doc_;

      GstElement *get_pipeline ();
	
    private:
      GstElement *pipeline_;
      GstBus *bus_;
      static gboolean bus_called (GstBus *bus,
				  GstMessage *msg,
				  gpointer data); 
    };

  
} // end of namespace

#endif // ifndef
