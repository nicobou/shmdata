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


#ifndef __SWITCHER_SHMDATA_WRITER_H__
#define __SWITCHER_SHMDATA_WRITER_H__

#include <memory>
#include <string>
#include <shmdata/base-writer.h>
//#include "gst-element-cleaner.h"
#include "json-builder.h"

namespace switcher
{

  class ShmdataWriter
  {
  public:
    typedef std::shared_ptr<ShmdataWriter> ptr;
    ShmdataWriter();
    ~ShmdataWriter();
    bool set_path (std::string name); //path needs to be fully specified
    bool set_path_without_deleting (std::string name); //path needs to be fully specified
    std::string get_path ();
    //caps does not need to be fully specified:
    void plug (GstElement *bin, GstElement *source_element,GstCaps *caps);
    void plug (GstElement *bin, GstPad *source_pad);
    //get json doc:
    JSONBuilder::Node get_json_root_node ();

  private:
    std::string path_;
    shmdata_base_writer_t *writer_;
    //GstElementCleaner::ptr cleaner_;
    GstElement *bin_;
    GstElement *tee_;
    GstElement *queue_;
    GstElement *fakesink_;
    JSONBuilder::ptr json_description_;
    void make_json_description ();
  };
  
}  // end of namespace

#endif // ifndef
