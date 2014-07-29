/*
 * This file is part of syphonsrc.
 *
 * switcher-top is free software; you can redistribute it and/or
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

#include "syphonsrc.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

using namespace std;
using namespace switcher::data;

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SyphonSrc,
				       "Video capture (through Syphon)",
				       "SyphonSrc", 
				       "Reads video input from a Syphon source",
				       "LGPL",
				       "syphonsrc",				
				       "Emmanuel Durand");

  SyphonSrc::SyphonSrc () :
    custom_props_(std::make_shared<CustomPropertyHelper> ()),
    syphon_servername_(""),
    syphon_servername_prop_(nullptr),
    syphon_appname_(""),
    syphon_appname_prop_(nullptr),
    width_(0),
    height_(0)
  {
  }
  
  SyphonSrc::~SyphonSrc ()
  {
  }

  bool SyphonSrc::init_gpipe()
  {
    init_startable (this);
    init_segment (this);

    reader_.reset(new SyphonReader(frameCallback, (void*)this));

    syphon_servername_prop_ = custom_props_->make_string_property ("servername",
        "servername is the name of the Syphon server",
        syphon_servername_.c_str(),
        (GParamFlags)G_PARAM_READWRITE,
        SyphonSrc::set_servername,
        SyphonSrc::get_servername,
        this);
    syphon_appname_prop_ = custom_props_->make_string_property ("appname",
        "appname is the name of the Syphon application",
        syphon_appname_.c_str(),
        (GParamFlags)G_PARAM_READWRITE,
        SyphonSrc::set_appname,
        SyphonSrc::get_appname,
        this);
    install_property_by_pspec (custom_props_->get_gobject (),
      syphon_servername_prop_,
      "servername",
      "Syphon server name");
    install_property_by_pspec (custom_props_->get_gobject (),
      syphon_appname_prop_,
      "appname",
      "Syphon application name");

    return true;
  }

  bool
  SyphonSrc::start()
  {
    if (syphon_servername_ == "" && syphon_appname_ == "")
    {
      cout << "SyphonSrc::start - No servername nor appname specified, using the first available server" << endl;
      reader_->connect(nullptr, nullptr);
    }
    else if (syphon_servername_ != "" && syphon_appname_ == "")
      reader_->connect(syphon_servername_.c_str(), nullptr);
    else if (syphon_servername_ == "" && syphon_appname_ != "")
      reader_->connect(nullptr, syphon_appname_.c_str());
    else
      reader_->connect(syphon_servername_.c_str(), syphon_appname_.c_str());

    return true;
  }

  bool
  SyphonSrc::stop()
  {
    reader_->disconnect();
    return true;
  }

  void
  SyphonSrc::frameCallback(void* context, const char* data, int& width, int& height)
  {
    SyphonSrc* ctx = (SyphonSrc*)context;

    char buffer[256] = "";
    static bool set = false;
    
    if (set == false || ctx->width_ != width || ctx->height_ != height)
    {
      ctx->writer_.reset(new ShmdataAnyWriter);
      if (ctx->syphon_servername_ != "" && ctx->syphon_appname_ != "")
        ctx->writer_->set_path(ctx->make_file_name(ctx->syphon_servername_ + "_" + ctx->syphon_appname_));
      else if (ctx->syphon_servername_ != "")
        ctx->writer_->set_path(ctx->make_file_name(ctx->syphon_servername_));
      else
        ctx->writer_->set_path(ctx->make_file_name(ctx->syphon_appname_));
      ctx->register_shmdata(ctx->writer_);
      ctx->width_ = width;
      ctx->height_ = height;

      sprintf(buffer, "video/x-raw-rgb,bpp=32,endianness=4321,depth=32,red_mask=-16777216,green_mask=16711680,blue_mask=65280,width=%i,height=%i,framerate=30/1", width, height);
      ctx->writer_->set_data_type(string(buffer));
      ctx->writer_->start();
      set = true;
    }

    ctx->writer_->push_data_auto_clock((void*)data, width * height * 4, nullptr, nullptr);
  }

  const gchar*
  SyphonSrc::get_servername(void* user_data)
  {
    SyphonSrc* ctx = (SyphonSrc*)user_data;
    return ctx->syphon_servername_.c_str();
  }

  void
  SyphonSrc::set_servername(const gchar* name, void* user_data)
  {
    SyphonSrc* ctx = (SyphonSrc*)user_data;

    if (name != nullptr)
      ctx->syphon_servername_ = name;
  }

  const gchar*
  SyphonSrc::get_appname(void* user_data)
  {
    SyphonSrc* ctx = (SyphonSrc*)user_data;
    return ctx->syphon_appname_.c_str();
  }

  void
  SyphonSrc::set_appname(const gchar* name, void* user_data)
  {
    SyphonSrc* ctx = (SyphonSrc*)user_data;

    if (name != nullptr)
      ctx->syphon_appname_ = name;
  }
}
