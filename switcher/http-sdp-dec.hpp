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

#ifndef __SWITCHER_HTTP_SDP_DEC_H__
#define __SWITCHER_HTTP_SDP_DEC_H__

#include <map>
#include <memory>
#include <string>
#include "./counter-map.hpp"
#include "./decodebin-to-shmdata.hpp"
#include "./g-source-wrapper.hpp"
#include "./gst-pipeliner.hpp"
#include "./gst-shmdata-subscriber.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
class GstShmdataSubscriber;

class HTTPSDPDec : public Quiddity {
 public:
  HTTPSDPDec(QuiddityConfiguration&&);

 private:
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  UGstElem souphttpsrc_;
  UGstElem sdpdemux_;
  bool is_dataurisrc_{false};
  guint retry_delay_{1000};
  bool decompress_streams_{true};
  PContainer::prop_id_t decompress_streams_id_;
  // will maintain a max of two GSourceWrapper in order to avoid destructing
  // itself from inside the GSource
  std::list<GSourceWrapper::uptr> on_error_{};
  std::string uri_{};
  std::list<std::unique_ptr<DecodebinToShmdata>> decodebins_{};
  std::string src_element_class_{"souphttpsrc"};
  CounterMap counter_{};
  std::vector<std::unique_ptr<GstShmdataSubscriber>> shm_subs_{};
  bool to_shmdata(std::string uri);
  void init_httpsdpdec();
  void destroy_httpsdpdec();
  void make_new_error_handler();
  void uri_to_shmdata();
  void configure_shmdatasink(GstElement* element,
                             const std::string& media_type,
                             const std::string& media_label);
  static void httpsdpdec_pad_added_cb(GstElement* object, GstPad* pad, gpointer user_data);
  static gboolean to_shmdata_wrapped(gpointer uri, gpointer user_data);
  static void on_new_element_in_sdpdemux(GstBin* bin, GstElement* element, gpointer user_data);
};

}  // namespace switcher
#endif
