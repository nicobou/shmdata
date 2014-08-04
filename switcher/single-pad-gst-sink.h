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


#ifndef __SWITCHER_BASE_SINK_H__
#define __SWITCHER_BASE_SINK_H__

#include "gpipe.h"
#include "shmdata-reader.h"
#include <memory>

namespace switcher
{

  class SinglePadGstSink : public GPipe
  {
  public:
    typedef std::shared_ptr<SinglePadGstSink> ptr;
    SinglePadGstSink ();
    ~SinglePadGstSink ();
    SinglePadGstSink (const SinglePadGstSink &) = delete;
    SinglePadGstSink &operator= (const SinglePadGstSink &) = delete;

    //  protected: //made public for allowing composition and/or delegation
    void set_sink_element (GstElement *sink);
    void set_sink_element_no_connect (GstElement *sink);
    void set_on_first_data_hook (ShmdataReader::on_first_data_hook cb, void *user_data);
    
  private:
    ShmdataReader::on_first_data_hook connection_hook_ {nullptr};
    void *hook_user_data_ {nullptr};
    GstElement *sink_element_ {nullptr};
    std::string shmdata_path_ {};

    //for segment
    bool connect (std::string shmdata_socket_path);
    bool disconnect_all ();

    //for subclasses
    virtual void on_shmdata_connect (std::string shmdata_sochet_path);
    virtual void on_shmdata_disconnect ();
    virtual bool can_sink_caps (std::string caps);
  };

}  // end of namespace

#endif // ifndef
