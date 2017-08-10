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

#include "./syphonsrc.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

using namespace std;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SyphonSrc,
                                     "syphonsrc",
                                     "Video capture (through Syphon)",
                                     "video",
                                     "writer/device",
                                     "Reads video input from a Syphon source",
                                     "LGPL",
                                     "Emmanuel Durand");

SyphonSrc::SyphonSrc(QuiddityConfiguration&&) {
  init_startable(this);

  reader_.reset(new SyphonReader(frameCallback, (void*)this));

  pmanage<MPtr(&PContainer::make_string)>("servername",
                                          [this](const std::string& val) {
                                            syphon_servername_ = val;
                                            return true;
                                          },
                                          [this]() { return syphon_servername_; },
                                          "Server name",
                                          "The name of the Syphon server",
                                          syphon_servername_);

  pmanage<MPtr(&PContainer::make_string)>("appname",
                                          [this](const std::string& val) {
                                            syphon_appname_ = val;
                                            return true;
                                          },
                                          [this]() { return syphon_appname_; },
                                          "App name",
                                          "The name of the Syphon application",
                                          syphon_appname_);
}

bool SyphonSrc::start() {
  if (syphon_servername_ == "" && syphon_appname_ == "") {
    cout << "SyphonSrc::start - No servername nor appname specified, using the "
            "first available server"
         << endl;
    reader_->connect(nullptr, nullptr);
  } else if (syphon_servername_ != "" && syphon_appname_ == "")
    reader_->connect(syphon_servername_.c_str(), nullptr);
  else if (syphon_servername_ == "" && syphon_appname_ != "")
    reader_->connect(nullptr, syphon_appname_.c_str());
  else
    reader_->connect(syphon_servername_.c_str(), syphon_appname_.c_str());

  return true;
}

bool SyphonSrc::stop() {
  reader_->disconnect();
  return true;
}

void SyphonSrc::frameCallback(void* context, const char* data, int& width, int& height) {
  SyphonSrc* ctx = static_cast<SyphonSrc*>(context);
  static bool set = false;
  if (set == false || ctx->width_ != width || ctx->height_ != height) {
    std::string writer_path;
    if (ctx->syphon_servername_ != "" && ctx->syphon_appname_ != "")
      writer_path = ctx->make_file_name(ctx->syphon_servername_ + "-" + ctx->syphon_appname_);
    else if (ctx->syphon_servername_ != "")
      writer_path = ctx->make_file_name(ctx->syphon_servername_);
    else
      writer_path = ctx->make_file_name(ctx->syphon_appname_);
    ctx->writer_ =
        std::make_unique<ShmdataWriter>(ctx,
                                        writer_path,
                                        width * height * 4,
                                        string("video/x-raw, format=RGBA, ") + "width=" +
                                            to_string(width) + "height=" + to_string(height));
    ctx->width_ = width;
    ctx->height_ = height;
    if (!ctx->writer_.get()) {
      warning("syphon to shmdata failed to start");
      ctx->writer_.reset(nullptr);
    } else
      set = true;
  }
  ctx->writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(static_cast<const void*>(data),
                                                            width * height * 4);
  ctx->writer_->bytes_written(width * height * 4);
}

}  // namespace switcher
