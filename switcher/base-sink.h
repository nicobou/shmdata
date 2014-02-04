/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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


#ifndef __SWITCHER_BASE_SINK_H__
#define __SWITCHER_BASE_SINK_H__

#include "segment.h"
#include "shmdata-reader.h"
#include <memory>

namespace switcher
{

  class BaseSink : public Segment
  {
  public:
    typedef std::shared_ptr<BaseSink> ptr;
    BaseSink ();
    ~BaseSink ();
    BaseSink (const BaseSink &) = delete;
    BaseSink &operator= (const BaseSink &) = delete;
    
  protected:
    void set_sink_element (GstElement *sink);
    void set_on_first_data_hook (ShmdataReader::on_first_data_hook cb, void *user_data);

  private:
    ShmdataReader::on_first_data_hook connection_hook_;
    void *hook_user_data_;
    GstElement *sink_element_;
    std::string shmdata_path_;
    bool connect (std::string shmdata_socket_path);
    //wrapper for being called
    static gboolean connect_wrapped (gpointer shmdata_socket_path, gpointer user_data);
    static gboolean disconnect (gpointer , gpointer user_data);
    virtual void on_shmdata_connect (std::string /* shmdata_sochet_path */) {};
    virtual void on_shmdata_disconnect () {};
  };

}  // end of namespace

#endif // ifndef
