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


#ifndef __SWITCHER_FAKE_SHMDATA_WRITER_H__
#define __SWITCHER_FAKE_SHMDATA_WRITER_H__

#include <memory>
#include "base-source.h"
#include "startable-quiddity.h"
#include "custom-property-helper.h"

namespace switcher
{

  class FakeShmdataWriter : public BaseSource, StartableQuiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(FakeShmdataWriter);
    FakeShmdataWriter ();
    ~FakeShmdataWriter ();
    FakeShmdataWriter (const FakeShmdataWriter &) = delete;
    FakeShmdataWriter &operator= (const FakeShmdataWriter &) = delete;

    bool add_shmdata_path (std::string name);
    bool start ();
    bool stop ();

  private:
    //custom properties:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *shmdata_path_spec_;
    gchar *shmdata_path_;
    bool clean ();
    bool init_segment ();
    static gboolean add_shmdata_path_wrapped (gpointer name, gpointer user_data);
    static void set_shmdata_path (const gchar *value, void *user_data);
    static gchar *get_shmdata_path (void *user_data);
  };

}  // end of namespace

#endif // ifndef
