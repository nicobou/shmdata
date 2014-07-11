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
    syphon_servername_prop_(NULL)
  {
  }
  
  SyphonSrc::~SyphonSrc ()
  {
  }

  bool SyphonSrc::init_segment()
  {
    syphon_servername_prop_ = custom_props_->make_string_property ("servername",
        "servername is the name of the Syphon server",
        syphon_servername_.c_str(),
        (GParamFlags)G_PARAM_READWRITE,
        SyphonSrc::set_servername,
        SyphonSrc::get_servername,
        this);
    install_property_by_pspec (custom_props_->get_gobject (),
      syphon_servername_prop_,
      "servername",
      "Syphon server name");

    return true;
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
    {
      string newName = name;
      if (ctx->syphon_servername_ != newName)
      {
        ctx->syphon_servername_ = newName;
        ctx->reader_.connect(ctx->syphon_servername_.c_str());
      }
    }
  }
}
