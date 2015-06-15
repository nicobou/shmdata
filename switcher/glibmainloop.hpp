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

#ifndef __SWITCHER_GLIB_MAINLOOP_H__
#define __SWITCHER_GLIB_MAINLOOP_H__

#include <glib.h>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace switcher {

class GlibMainLoop
{
 public:
  typedef std::shared_ptr<GlibMainLoop> ptr;
  GlibMainLoop();
  ~GlibMainLoop();
  GlibMainLoop(const GlibMainLoop &) = delete;
  GlibMainLoop &operator=(const GlibMainLoop &) = delete;

  GMainContext *get_main_context();
  
 private:
  GMainContext *main_context_{nullptr};
  GMainLoop *mainloop_{nullptr};
  std::mutex begin_{};
  std::mutex end_{};
  std::condition_variable end_cond_{};
  std::thread thread_;  // this runs the main loop
  void main_loop_thread();
};
}  // namespace switcher

#endif
