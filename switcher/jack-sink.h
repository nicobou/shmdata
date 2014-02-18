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


#ifndef __SWITCHER_JACK_SINK_H__
#define __SWITCHER_JACK_SINK_H__

#include "switcher/audio-sink.h"
#include "switcher/startable-quiddity.h"
#include <memory>

namespace switcher
{

  class JackSink : public AudioSink, public StartableQuiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(JackSink);
    JackSink ();
    ~JackSink ();
    JackSink (const JackSink &) = delete;
    JackSink &operator= (const JackSink &) = delete;

    bool start ();
    bool stop ();


  private:
    GstElement *jacksink_;
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *client_name_spec_;
    gchar *client_name_;
    bool init_segment ();
    bool make_elements ();
    void on_shmdata_disconnect ();
    void on_shmdata_connect (std::string /* shmdata_sochet_path */) ;
    static void set_client_name (const gchar *value, void *user_data);
    static const gchar *get_client_name (void *user_data);
  };

}  // end of namespace

#endif // ifndef
