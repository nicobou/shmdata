/*
 * This file is part of switcher-jack.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_JACK_CLIENT_H__
#define __SWITCHER_JACK_CLIENT_H__

#include <jack/jack.h>
#include <memory>
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class ShmdataToJack;
class JackClient : public SafeBoolIdiom {
  friend ShmdataToJack;
 public:
  explicit JackClient(const char *name);
  JackClient() = delete;
  jack_nframes_t get_sample_rate();
  jack_nframes_t get_buffer_size();
  void set_jack_process_callback(JackProcessCallback cb,
                                 void *arg);
  
 private:
  using jack_client_handle =
      std::unique_ptr<jack_client_t, decltype(&jack_client_close)>;
  jack_client_handle client_;
  jack_status_t status_{};
  jack_nframes_t sample_rate_{0};  // actually uint32_t
  jack_nframes_t buffer_size_{0};  // actually uint32_t
  JackProcessCallback user_cb_{nullptr};
  void *user_cb_arg_{nullptr};
  bool safe_bool_idiom() const final;
  static void on_jack_shutdown (void *arg);
  static int jack_process (jack_nframes_t nframes, void *arg);
  jack_client_t *get_raw();
};

}  // namespace switcher

#endif
