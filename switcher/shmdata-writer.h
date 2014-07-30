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


#ifndef __SWITCHER_SHMDATA_WRITER_H__
#define __SWITCHER_SHMDATA_WRITER_H__

#include <memory>
#include <string>
#include <shmdata/base-writer.h>
#include "json-builder.h"
#include "on-caps.h"

namespace switcher
{

  class ShmdataWriter : public OnCaps
  {
  public:
    typedef std::shared_ptr<ShmdataWriter> ptr;
    using CapsCallBack = std::function<void(std::string)>;
    ShmdataWriter();
    ~ShmdataWriter();
    ShmdataWriter (const ShmdataWriter &) = delete;
    ShmdataWriter &operator= (const ShmdataWriter &) = delete;

    bool set_path (std::string name); //path needs to be fully specified
    bool set_path_without_deleting (std::string name); //path needs to be fully specified
    std::string get_path ();

    //caps does not need to be fully specified:
    void plug (GstElement *bin, GstElement *source_element,GstCaps *caps);
    void plug (GstElement *bin, GstPad *source_pad);

    //get json doc:
    JSONBuilder::Node get_json_root_node ();

  private:
    std::string path_ {};
    shmdata_base_writer_t *writer_ {shmdata_base_writer_init ()};
    GstElement *bin_  {nullptr};
    GstElement *tee_ {nullptr};
    GstElement *queue_ {nullptr};
    GstElement *fakesink_ {nullptr};
    gulong handoff_handler_ {0};
    JSONBuilder::ptr json_description_ {new JSONBuilder()};

    void make_json_description ();
    static void on_handoff_cb (GstElement* object, 
			       GstBuffer* buf, 
			       GstPad* pad, 
			       gpointer user_data); 
  };
  
}  // end of namespace

#endif // ifndef
