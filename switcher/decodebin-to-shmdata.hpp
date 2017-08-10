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

#ifndef __SWITCHER_DECODEBIN_TO_SHMDATA_H__
#define __SWITCHER_DECODEBIN_TO_SHMDATA_H__

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include "./gst-pipeliner.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
// this class has been designed for being possessed by a gpipe

class DecodebinToShmdata {
  using on_configure_t = std::function<void(
      GstElement*, const std::string& /*media_type*/, const std::string& /*media_label*/)>;
  using on_buffer_discarded_t = std::function<void()>;

 public:
  explicit DecodebinToShmdata(GstPipeliner* gpipe,
                              on_configure_t on_gstshm_configure,
                              on_buffer_discarded_t on_buffer_discarded,
                              bool decompress);
  ~DecodebinToShmdata() = default;
  DecodebinToShmdata() = delete;
  DecodebinToShmdata(const DecodebinToShmdata&) = delete;
  DecodebinToShmdata& operator=(const DecodebinToShmdata&) = delete;

  // invoke a std::function on the internal decodebin as GstElement
  template <typename Return_type>
  Return_type invoke_with_return(std::function<Return_type(GstElement*)> command) {
    std::unique_lock<std::mutex> lock(thread_safe_);
    return decodebin_.invoke_with_return<Return_type>(command);
  }
  void invoke(std::function<void(GstElement*)> command);
  void set_media_label(std::string label);

 private:
  bool discard_next_uncomplete_buffer_{false};
  GstPad* main_pad_{nullptr};
  std::map<std::string, uint> media_counters_{};
  std::mutex media_counter_mutex_{};
  GstPipeliner* gpipe_;
  bool decompress_;
  on_configure_t on_gstshm_configure_;
  on_buffer_discarded_t on_buffer_discarded_;
  std::list<std::string> shmdata_path_{};  // for unregistering in the segment
  std::vector<gulong> cb_ids_{};
  std::mutex thread_safe_{};
  std::string media_label_{};
  UGstElem decodebin_;
  static void on_pad_added(GstElement* object, GstPad* pad, gpointer user_data);
  static int /*GstAutoplugSelectResult*/ on_autoplug_select(
      GstElement* bin, GstPad* pad, GstCaps* caps, GstElementFactory* factory, gpointer user_data);
  static GstPadProbeReturn gstrtpdepay_buffer_probe_cb(GstPad* /*pad */,
                                                       GstPadProbeInfo* /*info*/,
                                                       gpointer user_data);
  static GstPadProbeReturn gstrtpdepay_event_probe_cb(GstPad* /*pad */,
                                                      GstPadProbeInfo* /*info*/,
                                                      gpointer user_data);
  void pad_to_shmdata_writer(GstElement* bin, GstPad* pad);
  static gboolean eos_probe_cb(GstPad* pad, GstEvent* event, gpointer user_data);
  static gboolean rewind(gpointer user_data);
};

}  // namespace switcher
#endif
