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

#include "./g-source-wrapper.hpp"
#include <future>
#include "./gst-utils.hpp"

namespace switcher {
GSourceWrapper::GSourceWrapper(callback&& cb, bool async_invocation)
    : cb_(std::move(cb)), async_invocation_(async_invocation), gsource_(g_idle_source_new()) {
  g_source_set_priority(gsource_, G_PRIORITY_DEFAULT_IDLE);
  g_source_set_callback(gsource_,
                        (GSourceFunc)&GSourceWrapper::source_func,
                        (gpointer) this,
                        nullptr);  // GDestroyNotify
}

GSourceWrapper::GSourceWrapper(callback&& cb, guint delay_ms, bool async_invocation)
    : cb_(std::move(cb)),
      async_invocation_(async_invocation),
      gsource_(g_timeout_source_new(delay_ms)) {
  g_source_set_callback(gsource_, (GSourceFunc)&GSourceWrapper::source_func, this, nullptr);
}

bool GSourceWrapper::attach(GMainContext* gcontext) {
  if (attached_) {
    return false;
  }
  if (0 != g_source_attach(gsource_, gcontext)) attached_ = true;
  return attached_;
}

GSourceWrapper::~GSourceWrapper() {
  if (nullptr == gsource_) return;
  g_source_unref(gsource_);
  if (!g_source_is_destroyed(gsource_)) g_source_destroy(gsource_);
}

gboolean GSourceWrapper::source_func(gpointer user_data) {
  GSourceWrapper* context = static_cast<GSourceWrapper*>(user_data);
  if (!context->async_invocation_)
    context->cb_();
  else
    context->fut_ = std::async(std::launch::async, context->cb_);
  return FALSE;  // do not repeat in the glib mainloop
}
}  // namespace switcher
