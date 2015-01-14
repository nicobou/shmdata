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

#include <memory>
#include <map>
#include <string>
#include "./gst-pipeliner.hpp"
#include "./quiddity-command.hpp"
#include "./decodebin-to-shmdata.hpp"
#include "./g-source-wrapper.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
class HTTPSDPDec: public GstPipeliner {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(HTTPSDPDec);
  HTTPSDPDec(const std::string &);

 private:
  UGstElem souphttpsrc_;
  UGstElem sdpdemux_;
  guint retry_delay_{1000};
  // will maintain a max of two GSourceWrapper in order to avoid destructing
  // itself from inside the GSource 
  std::list<GSourceWrapper::uptr> on_error_{};
  std::string uri_{};
  std::list<std::unique_ptr<DecodebinToShmdata>> decodebins_{};
  bool to_shmdata(std::string uri);
  void init_httpsdpdec();
  void destroy_httpsdpdec();
  void make_new_error_handler();
  bool init_gpipe() final;
  void uri_to_shmdata();
  static void httpsdpdec_pad_added_cb(GstElement *object,
                                      GstPad *pad, gpointer user_data);
  static gboolean to_shmdata_wrapped(gpointer uri, gpointer user_data);
  static void source_setup_cb(GstElement *httpsdpdec,
                              GstElement *source, gpointer user_data);
  static void on_new_element_in_sdpdemux(GstBin *bin,
                                         GstElement *element,
                                         gpointer user_data);
};
}  // namespace switcher

#endif
