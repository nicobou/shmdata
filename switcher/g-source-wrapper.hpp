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

#ifndef __SWITCHER_G_SOURCE_WRAPPER_H__
#define __SWITCHER_G_SOURCE_WRAPPER_H__

#include <memory>
#include <future>
#include <glib.h>

namespace switcher {
class GSourceWrapper {
 public:
  using callback = std::function<void()>;
  using uptr = std::unique_ptr<GSourceWrapper>;

  // for imediate invocation
  GSourceWrapper(callback &&cb,
                 bool async_invocation = false);
  // for delayed invocation
  GSourceWrapper(callback &&cb,
                 guint delay_ms,
                 bool async_invocation = false);
  bool attach(GMainContext *gcontext);
  
  GSourceWrapper() = delete;
  ~GSourceWrapper();
  GSourceWrapper(const GSourceWrapper &) = delete;
  GSourceWrapper &operator=(const GSourceWrapper &) = delete;

  
 private:
  callback cb_;
  bool attached_{false};
  bool async_invocation_;
  std::future<void> fut_{};
  GSource *gsource_{nullptr};  // a valid ID is greater than 0
  //the GSourceFunc that will be passed to glib
  static gboolean source_func(gpointer user_data);
};

}  // namespace switcher
#endif
