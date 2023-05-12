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

/**
 * The Pipeliner class
 */

#include "./pipeliner.hpp"
#include <algorithm>
#include "../quiddity/quiddity.hpp"
#include "../utils/scope-exit.hpp"
#include "./g-source-wrapper.hpp"
#include "./utils.hpp"

namespace switcher {
namespace gst {
Pipeliner::Pipeliner(Pipe::on_msg_async_cb_t on_msg_async_cb, Pipe::on_msg_sync_cb_t on_msg_sync_cb)
    : Pipeliner(on_msg_async_cb, on_msg_sync_cb, nullptr) {}


Pipeliner::Pipeliner(GMainContext* context) :
    Pipeliner(nullptr, nullptr, nullptr, context) {}
    
Pipeliner::Pipeliner(Pipe::on_msg_async_cb_t on_msg_async_cb,
                     Pipe::on_msg_sync_cb_t on_msg_sync_cb,
                     on_error_cb_t on_error_cb)
    : Pipeliner(on_msg_async_cb, on_msg_sync_cb, on_error_cb, nullptr) {}

Pipeliner::Pipeliner(Pipe::on_msg_async_cb_t on_msg_async_cb,
                     Pipe::on_msg_sync_cb_t on_msg_sync_cb,
                     on_error_cb_t on_error_cb,
                     GMainContext* context)
    : on_msg_async_cb_(on_msg_async_cb),
      on_msg_sync_cb_(on_msg_sync_cb),
      on_error_cb_(on_error_cb),
      main_loop_(context ? nullptr : std::make_unique<GlibMainLoop>()),
      context_(context ? context : main_loop_->get_main_context()),
      gst_pipeline_(std::make_unique<Pipe>(
          context_, &Pipeliner::bus_sync_handler, this
          // [this](GstMessage *msg){
          //   if(this->on_msg_async_cb_)
          //     this->on_msg_async_cb_(msg);
          // },
          // [this](GstMessage *msg){
          //   if (GST_BUS_DROP == this->on_gst_error(msg))
          //     return GST_BUS_DROP;
          //   else {
          //     if (this->on_msg_sync_cb_)
          //       return this->on_msg_sync_cb_(msg);
          //     else
          //       return GST_BUS_PASS;
          //   }
          // }
          )) {
  if (!gst_pipeline_) {
    return;
  }

  gst::utils::g_idle_add_full_with_context(
      context_, G_PRIORITY_DEFAULT_IDLE, push_thread_context, this, nullptr);
}

Pipeliner::~Pipeliner() {
  std::unique_lock<std::mutex> lock(watch_mutex_);
  while (!watch_added_) cond_watch_.wait_for(lock, std::chrono::milliseconds(200));
  auto bus = gst_pipeline_get_bus(GST_PIPELINE(get_pipeline()));
  On_scope_exit { gst_object_unref(bus); };
  gst_bus_remove_watch(bus);
}

gboolean Pipeliner::push_thread_context(gpointer user_data) {
  auto context = static_cast<Pipeliner*>(user_data);
  g_main_context_push_thread_default(context->context_);

  std::unique_lock<std::mutex> lock(context->watch_mutex_);
  auto bus = gst_pipeline_get_bus(GST_PIPELINE(context->get_pipeline()));
  On_scope_exit { gst_object_unref(bus); };
  gst_bus_add_watch(bus, bus_watch, user_data);
  context->watch_added_ = true;
  context->cond_watch_.notify_one();
  return FALSE;
}

void Pipeliner::play(gboolean play) {
  if (play) {
    gst_pipeline_->play(true);
  } else {
    gst_pipeline_->play(false);
  }
}

bool Pipeliner::speed(double speed) { return gst_pipeline_->speed(speed); }

bool Pipeliner::seek(gdouble position_in_ms) { return gst_pipeline_->seek(position_in_ms); }

bool Pipeliner::seek_key_frame(gdouble position_in_ms) {
  return gst_pipeline_->seek_key_frame(position_in_ms);
};

GstElement* Pipeliner::get_pipeline() { return gst_pipeline_->get_pipeline(); }

GMainContext* Pipeliner::get_main_context() const { return context_; }

GstBusSyncReply Pipeliner::on_gst_error(GstMessage* msg) {
  if (GST_MESSAGE_TYPE(msg) != GST_MESSAGE_ERROR) return GST_BUS_PASS;
  gchar* debug = nullptr;
  GError* error = nullptr;
  gst_message_parse_error(msg, &error, &debug);
  g_free(debug);
  On_scope_exit {
    if (error) g_error_free(error);
  };
  // on-error-gsource
  GSourceWrapper* gsrc =
      static_cast<GSourceWrapper*>(g_object_get_data(G_OBJECT(msg->src), "on-error-gsource"));
  if (nullptr != gsrc) {
    // removing command in order to get it invoked once
    g_object_set_data(G_OBJECT(msg->src), "on-error-gsource", nullptr);
    gsrc->attach(context_);
  }
  if (on_error_cb_) on_error_cb_(GST_MESSAGE_SRC(msg), error);
  return GST_BUS_DROP;
}

gboolean Pipeliner::bus_watch(GstBus* /*bus*/, GstMessage* message, gpointer user_data) {
  auto context = static_cast<Pipeliner*>(user_data);
  if (context) {
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
      if (context->loop_) context->seek(0);
    }
  }
  if (context && context->on_msg_async_cb_) {
    context->on_msg_async_cb_(message);
  }
  return TRUE;
}

GstBusSyncReply Pipeliner::bus_sync_handler(GstBus* /*bus*/,
                                               GstMessage* msg,
                                               gpointer user_data) {
  Pipeliner* context = static_cast<Pipeliner*>(user_data);
  auto res = GST_BUS_PASS;
  if (GST_BUS_DROP == context->on_gst_error(msg))
    return GST_BUS_DROP;
  else {
    if (context->on_msg_sync_cb_) {
      if (GST_BUS_DROP == context->on_msg_sync_cb_(msg)) return GST_BUS_DROP;
    }
  }

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    gchar* debug = nullptr;
    GError* error = nullptr;
    gst_message_parse_error(msg, &error, &debug);
    g_free(debug);
    g_error_free(error);
    res = GST_BUS_DROP;
  }

  return res;
}

void Pipeliner::loop(bool looping) { loop_ = looping; }

}  // namespace gst
}  // namespace switcher
