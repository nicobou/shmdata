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

ShmdataDecoder::ShmdataDecoder(Quiddity *quid,
                               GstPipeliner *pipeliner,
                               const std::string &shmpath,
                               const std::string &shm_prefix,
                               const std::string &media_label,
                               on_shmwriter_path_t cb):
    quid_(quid),
    pipeliner_(pipeliner),
    shmpath_(shmpath),
    shm_prefix_(shm_prefix),
    media_label_(media_label),
    shmdatasrc_(gst_element_factory_make("shmdatasrc", nullptr)),
    decodebin_(
        pipeliner_,
        [this](GstElement *el, const std::string &media_type, const std::string &media_label){
          configure_shmdatasink(el, media_type, media_label);
        }),
  on_shmwriter_path_cb_(cb){
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (nullptr == shmdatasrc_){
    g_warning("ShmdataDecoder failed to create GStreamer element");
    return;
  }
    g_print("%s %d\n", __FUNCTION__, __LINE__);

  // configuring shmdatasrc
  g_object_set(G_OBJECT(shmdatasrc_),
               "socket-path", shmpath_.c_str(),
               "copy-buffers", TRUE,
               nullptr);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  gst_bin_add_many(GST_BIN(pipeliner_->get_pipeline()), shmdatasrc_, nullptr);
  // HERE link shmdatasrc with the decodebin, as done in httpsdpdec, then use it in pj-call
  // if (!gst_element_link(shmdatasrc_, typefind_))
  //   return;
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if(!decodebin_.invoke_with_return<gboolean>([&](GstElement *el) {
        if (!gst_bin_add(GST_BIN(pipeliner_->get_pipeline()), el)) {
          g_warning("decodebin cannot be added to pipeline");
          return false;
        }
        if (!gst_element_link(shmdatasrc_, el)) {
          g_warning("decodebin cannot be linked to shmdatasrc");
          return false;
        }
        return true;
      })){
  }
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  
  // auto caps = gst_pad_get_allowed_caps(pad);
  // On_scope_exit{gst_caps_unref(caps);};
  // auto structure = gst_caps_get_structure(caps, 0);
  // auto media_label = gst_structure_get_string (structure, "media-label");
  // if (nullptr != media_label)
  //   decodebin->set_media_label(gst_structure_get_string (structure, "media-label"));
  if (!media_label_.empty())
    decodebin_.set_media_label(media_label_);
  g_print("%s %d\n", __FUNCTION__, __LINE__);

  decodebin_.invoke([](GstElement *el) { GstUtils::sync_state_with_parent(el); });
  GstUtils::sync_state_with_parent(shmdatasrc_);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
}

ShmdataDecoder::~ShmdataDecoder(){
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!shmwriter_path_.empty())
    quid_->prune_tree(std::string(".shmdata.writer." + shmwriter_path_));
  if (shmdatasrc_)
    GstUtils::clean_element(shmdatasrc_);
}

void ShmdataDecoder::configure_shmdatasink(GstElement *element,
                                           const std::string &media_type,
                                           const std::string &media_label){
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  std::string media_name = media_type;
  std::string shmpath;
  if (shm_prefix_.empty())
    shmpath =
        quid_->get_file_name_prefix() + quid_->get_manager_name() + "_" + quid_->get_name() + "_";
  else
    shmpath = shm_prefix_;
  
  if (media_label.empty())
    shmpath += media_name;
  else
    shmpath += media_label + "-" + media_name;
  shmwriter_path_ = shmpath;
  if (on_shmwriter_path_cb_)
    on_shmwriter_path_cb_(shmwriter_path_);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  g_object_set(G_OBJECT(element), "socket-path", shmpath.c_str(), nullptr);
  auto *quid = quid_;
  g_print("%s %d\n", __FUNCTION__, __LINE__);
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
  g_print("%s %d\n", __FUNCTION__, __LINE__);
}

}  // namespace switcher
