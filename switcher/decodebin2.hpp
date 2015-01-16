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

#ifndef __SWITCHER_DECODEBIN2_H__
#define __SWITCHER_DECODEBIN2_H__

#include <memory>
#include <map>
#include "./single-pad-gst-sink.hpp"
#include "./decodebin-to-shmdata.hpp"

namespace switcher {
class Decodebin2:public SinglePadGstSink {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(Decodebin2);
  Decodebin2();
  Decodebin2(const Decodebin2 &) = delete;
  Decodebin2 &operator=(const Decodebin2 &) = delete;

 private:
  CustomPropertyHelper::ptr custom_props_{};
  GParamSpec *media_label_spec_{nullptr};
  std::string media_label_{};
  std::unique_ptr<DecodebinToShmdata> decodebin_;
  std::map<std::string, int> media_counters_ {};
  bool init_gpipe() final;
  bool can_sink_caps(std::string /*caps*/) final {
    return true;
  };
  static void make_decodebin_active(ShmdataReader *caller,
                                    void *decodebin2_instance);
  static void set_media_label(const gchar *value, void *user_data);
  static const gchar *get_media_label(void *user_data);
};

}  // namespace switcher
#endif
