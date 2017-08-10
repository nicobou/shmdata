/*
 * This file is part of switcher-pulse.
 *
 * Partially from pactl.c Copyright 2004-2006 Lennart Poettering
 *
 * switcher-pulse is free software; you can redistribute it and/or
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

#include "./pulsesink.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/gst-utils.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSink,
                                     "pulsesink",
                                     "Audio Display (Pulse)",
                                     "audio",
                                     "reader/device",
                                     "Inspecting Devices And Playing Audio To Outputs",
                                     "LGPL",
                                     "Nicolas Bouillot");

PulseSink::PulseSink(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      mainloop_(std::make_unique<GlibMainLoop>()),
      shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)) {
  if (!shmsrc_ || !audioconvert_ || !pulsesink_) {
    is_valid_ = false;
    return;
  }
  g_object_set(G_OBJECT(pulsesink_.get_raw()),
               "slave-method",
               0,  // resample
               "client-name",
               get_name().c_str(),
               nullptr);
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);

  std::unique_lock<std::mutex> lock(devices_mutex_);
  GstUtils::g_idle_add_full_with_context(mainloop_->get_main_context(),
                                         G_PRIORITY_DEFAULT_IDLE,
                                         async_get_pulse_devices,
                                         this,
                                         nullptr);
  devices_cond_.wait(lock);
  if (!connected_to_pulse_) {
    message("ERROR:Not connected to pulse, cannot initialize.");
    is_valid_ = false;
    return;
  }
  volume_id_ = pmanage<MPtr(&PContainer::push)>(
      "volume", GPropToProp::to_prop(G_OBJECT(pulsesink_.get_raw()), "volume"));
  mute_id_ = pmanage<MPtr(&PContainer::push)>(
      "mute", GPropToProp::to_prop(G_OBJECT(pulsesink_.get_raw()), "mute"));
}

PulseSink::~PulseSink() {
  GMainContext* main_context = mainloop_->get_main_context();
  if (nullptr != main_context && connected_to_pulse_) {
    std::unique_lock<std::mutex> lock(quit_mutex_);
    GstUtils::g_idle_add_full_with_context(
        main_context, G_PRIORITY_DEFAULT_IDLE, quit_pulse, this, nullptr);
    quit_cond_.wait(lock);
  }
}

gboolean PulseSink::quit_pulse(void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  pa_context_disconnect(context->pa_context_);
  // pa_context_unref (context->pa_context_);
  // context->pa_context_ = nullptr;
  pa_glib_mainloop_free(context->pa_glib_mainloop_);
  std::unique_lock<std::mutex> lock(context->quit_mutex_);
  context->quit_cond_.notify_all();
  return FALSE;
}

gboolean PulseSink::async_get_pulse_devices(void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  context->pa_glib_mainloop_ = pa_glib_mainloop_new(context->mainloop_->get_main_context());
  context->pa_mainloop_api_ = pa_glib_mainloop_get_api(context->pa_glib_mainloop_);
  context->pa_context_ = pa_context_new(context->pa_mainloop_api_, nullptr);
  if (nullptr == context->pa_context_) {
    context->debug("PulseSink:: pa_context_new() failed.");
    return FALSE;
  }
  pa_context_set_state_callback(context->pa_context_, pa_context_state_callback, context);
  if (pa_context_connect(context->pa_context_, context->server_, (pa_context_flags_t)0, nullptr) <
      0) {
    context->debug("pa_context_connect() failed: %",
                   std::string(pa_strerror(pa_context_errno(context->pa_context_))));
    return FALSE;
  }
  context->connected_to_pulse_ = true;
  return FALSE;
}

bool PulseSink::remake_elements() {
  pmanage<MPtr(&PContainer::remove)>(volume_id_);
  volume_id_ = 0;
  pmanage<MPtr(&PContainer::remove)>(mute_id_);
  mute_id_ = 0;
  if (!UGstElem::renew(pulsesink_, {"volume", "mute", "slave-method", "client-name", "device"}) ||
      !UGstElem::renew(shmsrc_) || !UGstElem::renew(audioconvert_))
    return false;
  volume_id_ = pmanage<MPtr(&PContainer::push)>(
      "volume", GPropToProp::to_prop(G_OBJECT(pulsesink_.get_raw()), "volume"));
  mute_id_ = pmanage<MPtr(&PContainer::push)>(
      "mute", GPropToProp::to_prop(G_OBJECT(pulsesink_.get_raw()), "mute"));
  if (!devices_.empty())
    g_object_set(G_OBJECT(pulsesink_.get_raw()),
                 "device",
                 devices_.at(devices_enum_.get_current_index()).name_.c_str(),
                 nullptr);
  return true;
}

void PulseSink::pa_context_state_callback(pa_context* pulse_context, void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  switch (pa_context_get_state(pulse_context)) {
    case PA_CONTEXT_CONNECTING:
      // g_print ("PA_CONTEXT_CONNECTING\n");
      break;
    case PA_CONTEXT_AUTHORIZING:
      // g_print ("PA_CONTEXT_AUTHORIZING\n");
      break;
    case PA_CONTEXT_SETTING_NAME:
      // g_print ("PA_CONTEXT_SETTING_NAME\n");
      break;
    case PA_CONTEXT_READY:
      // g_print ("PA_CONTEXT_READY\n");
      context->make_device_description(pulse_context);
      pa_context_set_subscribe_callback(pulse_context, on_pa_event_callback, nullptr);
      pa_operation_unref(pa_context_subscribe(
          pulse_context,
          static_cast<pa_subscription_mask_t>(
              PA_SUBSCRIPTION_MASK_SINK | PA_SUBSCRIPTION_MASK_SOURCE |
              PA_SUBSCRIPTION_MASK_SINK_INPUT | PA_SUBSCRIPTION_MASK_SOURCE_OUTPUT |
              PA_SUBSCRIPTION_MASK_MODULE | PA_SUBSCRIPTION_MASK_CLIENT |
              PA_SUBSCRIPTION_MASK_SAMPLE_CACHE | PA_SUBSCRIPTION_MASK_SERVER |
              PA_SUBSCRIPTION_MASK_CARD),
          nullptr,    // pa_context_success_cb_t cb,
          nullptr));  // void *userdata
      break;
    case PA_CONTEXT_TERMINATED:
      pa_context_unref(context->pa_context_);
      context->pa_context_ = nullptr;
      break;
    case PA_CONTEXT_FAILED:
      break;
    default:
      context->debug("PulseSink Context error: %",
                     std::string(pa_strerror(pa_context_errno(pulse_context))));
  }
}

void PulseSink::get_sink_info_callback(pa_context* pulse_context,
                                       const pa_sink_info* i,
                                       int is_last,
                                       void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  if (is_last < 0) {
    context->debug("Failed to get sink information: %",
                   std::string(pa_strerror(pa_context_errno(pulse_context))));
    return;
  }
  if (is_last) {
    pa_operation* operation = pa_context_drain(pulse_context, nullptr, nullptr);
    if (operation) pa_operation_unref(operation);
    context->update_output_device();
    context->devices_enum_id_ = context->pmanage<MPtr(&PContainer::make_selection<>)>(
        "device",
        [context](const IndexOrName& val) {
          context->devices_enum_.select(val);
          return true;
        },
        [context]() { return context->devices_enum_.get(); },
        "Device",
        "Audio playback device to use",
        context->devices_enum_);
    std::unique_lock<std::mutex> lock(context->devices_mutex_);
    context->devices_cond_.notify_all();
    return;
  }
  DeviceDescription description;
  switch (i->state) {
    case PA_SINK_INIT:
      description.state_ = "INIT";
      // g_print ("state: INIT \n");
      break;
    case PA_SINK_UNLINKED:
      description.state_ = "UNLINKED";
      // g_print ("state: UNLINKED \n");
      break;
    case PA_SINK_INVALID_STATE:
      description.state_ = "n/a";
      // g_print ("state: n/a \n");
      break;
    case PA_SINK_RUNNING:
      description.state_ = "RUNNING";
      // g_print ("state: RUNNING \n");
      break;
    case PA_SINK_IDLE:
      description.state_ = "IDLE";
      // g_print ("state: IDLE \n");
      break;
    case PA_SINK_SUSPENDED:
      description.state_ = "SUSPENDED";
      // g_print ("state: SUSPENDED \n");
      break;
  }
  description.name_ = i->name;
  if (i->description == nullptr)
    description.description_ = "";
  else
    description.description_ = i->description;
  description.sample_format_ = pa_sample_format_to_string(i->sample_spec.format);
  gchar* rate = g_strdup_printf("%u", i->sample_spec.rate);
  description.sample_rate_ = rate;
  g_free(rate);
  gchar* channels = g_strdup_printf("%u", i->sample_spec.channels);
  description.channels_ = channels;
  g_free(channels);
  if (i->ports) {
    pa_sink_port_info** p;
    for (p = i->ports; *p; p++) {
      description.ports_.push_back(std::make_pair((*p)->name, (*p)->description));
    }
  }
  if (i->active_port) {
    description.active_port_ = i->active_port->description;
  } else
    description.active_port_ = "n/a";
  context->devices_.push_back(description);
}

void PulseSink::make_device_description(pa_context* pulse_context) {
  devices_.clear();
  pa_operation_unref(pa_context_get_sink_info_list(pulse_context, get_sink_info_callback, this));
}

void PulseSink::on_pa_event_callback(pa_context* pulse_context,
                                     pa_subscription_event_type_t pulse_event_type,
                                     uint32_t /*index */,
                                     void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SINK) {
    if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_NEW) {
      context->make_device_description(pulse_context);
      return;
    }
  }
  if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SINK) {
    if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_REMOVE) {
      context->make_device_description(pulse_context);
      return;
    }
  }
}
void PulseSink::update_output_device() {
  std::vector<std::string> names;
  std::vector<std::string> nicks;
  for (auto& it : devices_) {
    names.push_back(it.description_);
    nicks.push_back(it.name_);
  }
  devices_enum_ = Selection<>(std::make_pair(names, nicks), 0);
}

bool PulseSink::can_sink_caps(const std::string& caps) {
  return GstUtils::can_sink_caps("audioconvert", caps);
};

bool PulseSink::on_shmdata_disconnect() {
  pmanage<MPtr(&PContainer::enable)>(devices_enum_id_);
  prune_tree(".shmdata.reader." + shmpath_);
  shm_sub_.reset();
  On_scope_exit { gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr); };
  return remake_elements();
}

bool PulseSink::on_shmdata_connect(const std::string& shmpath) {
  pmanage<MPtr(&PContainer::disable)>(devices_enum_id_, ShmdataConnector::disabledWhenConnectedMsg);
  shmpath_ = shmpath;
  g_object_set(G_OBJECT(shmsrc_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  if (!devices_.empty())
    g_object_set(G_OBJECT(pulsesink_.get_raw()),
                 "device",
                 devices_.at(devices_enum_.get_current_index()).name_.c_str(),
                 nullptr);
  shm_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this](const std::string& caps) {
        this->graft_tree(
            ".shmdata.reader." + shmpath_,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.reader." + shmpath_));

  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmsrc_.get_raw(),
                   audioconvert_.get_raw(),
                   pulsesink_.get_raw(),
                   nullptr);
  gst_element_link_many(shmsrc_.get_raw(), audioconvert_.get_raw(), pulsesink_.get_raw(), nullptr);
  gst_pipeline_->play(true);
  return true;
}

}  // namespace switcher
