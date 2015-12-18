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

#include "./shmdata-decoder.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {

ShmdataDecoder::ShmdataDecoder(Quiddity *quid, GstPipeliner *pipeliner, const std::string &shmpath):
    quid_(quid),
    pipeliner_(pipeliner),
    shmpath_(shmpath),
    shmdatasrc_(gst_element_factory_make("shmdatasrc", nullptr)),
    decodebin_(
        pipeliner_,
        [this](GstElement *el, const std::string &media_type, const std::string &media_label){
          configure_shmdatasink(el, media_type, media_label);
        }) {
  if (nullptr == shmdatasrc_){
    g_warning("ShmdataDecoder failled to create GStreamer element");
    return;
  }
  
  // configuring shmdatasrc
  g_object_set(G_OBJECT(shmdatasrc_),
               "socket-path", shmpath_.c_str(),
               nullptr);
  gst_bin_add_many(GST_BIN(pipeliner_->get_pipeline()),
                   shmdatasrc_, nullptr);
  // HERE link shmdatasrc with the decodebin, as done in httpsdpdec, then use it in pj-call
  // if (!gst_element_link(shmdatasrc_, typefind_))
  //   return;
  GstUtils::sync_state_with_parent(shmdatasrc_);
}

ShmdataDecoder::~ShmdataDecoder(){
  if (shmdatasrc_)
    GstUtils::clean_element(shmdatasrc_);
}

void ShmdataDecoder::configure_shmdatasink(GstElement *element,
                                           const std::string &media_type,
                                           const std::string &media_label){
  std::string media_name = media_type;
  std::string shmpath;
  if (media_label.empty())
    shmpath = quid_->make_file_name(media_name);
  else
    shmpath = quid_->make_file_name(media_label + "-" + media_name);
  
  g_object_set(G_OBJECT(element), "socket-path", shmpath.c_str(), nullptr);
  auto *quid = quid_;
  shm_sub_ =
      std2::make_unique<GstShmdataSubscriber>(
          element,
          [quid, shmpath]( const std::string &caps){
            quid->graft_tree(".shmdata.writer." + shmpath,
                             ShmdataUtils::make_tree(caps,
                                                     ShmdataUtils::get_category(caps),
                                                     0));
          },
          [quid, shmpath](GstShmdataSubscriber::num_bytes_t byte_rate){
            quid->graft_tree(".shmdata.writer." + shmpath + ".byte_rate",
                             InfoTree::make(byte_rate));
          });
}

}  // namespace switcher
