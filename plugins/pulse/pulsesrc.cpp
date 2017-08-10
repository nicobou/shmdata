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

#include "./pulsesrc.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSrc,
                                     "pulsesrc",
                                     "Pulse Audio Device",
                                     "audio",
                                     "writer/device",
                                     "Audio From Pulse audio driver",
                                     "LGPL",
                                     "Nicolas Bouillot");

PulseSrc::PulseSrc(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      mainloop_(std::make_unique<GlibMainLoop>()),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)) {
  pmanage<MPtr(&PContainer::make_group)>(
      "advanced", "Advanced configuration", "Advanced configuration");
  pmanage<MPtr(&PContainer::make_parented_selection<>)>(
      "save_mode",
      "advanced",
      [this](const IndexOrName& val) {
        save_device_enum_.select(val);
        return true;
      },
      [this]() { return save_device_enum_.get(); },
      "Save Mode",
      "Save Audio Capture Device by device or by port.",
      save_device_enum_);
  init_startable(this);
  if (!pulsesrc_ || !shmsink_) {
    is_valid_ = false;
    return;
  }
  shmpath_ = make_file_name("audio");
  g_object_set(G_OBJECT(pulsesrc_.get_raw()), "client-name", get_name().c_str(), nullptr);
  g_object_set(G_OBJECT(shmsink_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  std::unique_lock<std::mutex> lock(devices_mutex_);
  GstUtils::g_idle_add_full_with_context(mainloop_->get_main_context(),
                                         G_PRIORITY_DEFAULT_IDLE,
                                         async_get_pulse_devices,
                                         this,
                                         nullptr);
  // waiting for devices to be updated
  devices_cond_.wait(lock);
  if (!connected_to_pulse_) {
    message("ERROR:Not connected to pulse, cannot initialize.");
    is_valid_ = false;
    return;
  }
  volume_id_ = pmanage<MPtr(&PContainer::push)>(
      "volume", GPropToProp::to_prop(G_OBJECT(pulsesrc_.get_raw()), "volume"));
  mute_id_ = pmanage<MPtr(&PContainer::push)>(
      "mute", GPropToProp::to_prop(G_OBJECT(pulsesrc_.get_raw()), "mute"));
}

gboolean PulseSrc::async_get_pulse_devices(void* user_data) {
  PulseSrc* context = static_cast<PulseSrc*>(user_data);
  context->pa_glib_mainloop_ = pa_glib_mainloop_new(context->mainloop_->get_main_context());
  context->pa_mainloop_api_ = pa_glib_mainloop_get_api(context->pa_glib_mainloop_);
  context->pa_context_ = pa_context_new(context->pa_mainloop_api_, nullptr);
  if (nullptr == context->pa_context_) {
    context->debug("PulseSrc:: pa_context_new() failed.");
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

PulseSrc::~PulseSrc() {
  GMainContext* main_context = mainloop_->get_main_context();
  if (nullptr != main_context && connected_to_pulse_) {
    std::unique_lock<std::mutex> lock(quit_mutex_);
    GstUtils::g_idle_add_full_with_context(
        main_context, G_PRIORITY_DEFAULT_IDLE, quit_pulse, this, nullptr);
    quit_cond_.wait(lock);
  }
}

gboolean PulseSrc::quit_pulse(void* user_data) {
  PulseSrc* context = static_cast<PulseSrc*>(user_data);
  pa_context_disconnect(context->pa_context_);
  // pa_context_unref (context->pa_context_);
  // context->pa_context_ = nullptr;
  pa_glib_mainloop_free(context->pa_glib_mainloop_);
  std::unique_lock<std::mutex> lock(context->quit_mutex_);
  context->quit_cond_.notify_all();
  return FALSE;
}

bool PulseSrc::remake_elements() {
  if (!UGstElem::renew(pulsesrc_, {"client-name", "volume", "mute", "device"}) ||
      !UGstElem::renew(shmsink_, {"socket-path"}))
    return false;
  return true;
}

void PulseSrc::pa_context_state_callback(pa_context* pulse_context, void* user_data) {
  PulseSrc* context = static_cast<PulseSrc*>(user_data);
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
      pa_context_set_subscribe_callback(pulse_context, on_pa_event_callback, context);
      pa_operation_unref(pa_context_subscribe(
          pulse_context,
          static_cast<pa_subscription_mask_t>(
              PA_SUBSCRIPTION_MASK_SINK | PA_SUBSCRIPTION_MASK_SOURCE |
              PA_SUBSCRIPTION_MASK_SINK_INPUT | PA_SUBSCRIPTION_MASK_SOURCE_OUTPUT |
              PA_SUBSCRIPTION_MASK_MODULE | PA_SUBSCRIPTION_MASK_CLIENT |
              PA_SUBSCRIPTION_MASK_SAMPLE_CACHE | PA_SUBSCRIPTION_MASK_SERVER |
              PA_SUBSCRIPTION_MASK_CARD),
          nullptr,    // pa_context_success_cb_t cb,
          nullptr));  // void *userdata);
      break;
    case PA_CONTEXT_TERMINATED: {
      context->debug("PulseSrc: PA_CONTEXT_TERMINATED");
    } break;
    case PA_CONTEXT_FAILED:
      context->debug("PA_CONTEXT_FAILED");
      break;
    default:
      context->debug("PulseSrc Context error: %",
                     std::string(pa_strerror(pa_context_errno(pulse_context))));
  }
}

void PulseSrc::get_source_info_callback(pa_context* pulse_context,
                                        const pa_source_info* i,
                                        int is_last,
                                        void* user_data) {
  PulseSrc* context = static_cast<PulseSrc*>(user_data);
  if (is_last < 0) {
    context->debug("Failed to get source information: %",
                   std::string(pa_strerror(pa_context_errno(pulse_context))));
    return;
  }
  if (is_last) {
    pa_operation* operation = pa_context_drain(pulse_context, nullptr, nullptr);
    if (operation) pa_operation_unref(operation);
    // registering enum for devices
    context->update_capture_device();

    auto set = [context](const IndexOrName& val) {
      if (context->is_loading_) return false;
      context->devices_.select(val);
      return true;
    };
    auto get = [context]() { return context->devices_.get_current_index(); };

    if (!context->devices_id_) {
      context->devices_id_ = context->pmanage<MPtr(&PContainer::make_selection<>)>(
          "device", set, get, "Device", "Audio capture device to use", context->devices_);
    } else {
      context->pmanage<MPtr(&PContainer::replace_and_notify)>(
          context->devices_id_,
          std::make_unique<Property<Selection<>, Selection<>::index_t>>(
              set,
              get,
              "Device",
              "Audio capture device to use",
              context->devices_,
              context->devices_.size() - 1));
    }

    context->devices_cond_.notify_all();
    return;
  }

  DeviceDescription description;
  switch (i->state) {
    case PA_SOURCE_INIT:
      description.state_ = "INIT";
      // g_print ("state: INIT \n");
      break;
    case PA_SOURCE_UNLINKED:
      description.state_ = "UNLINKED";
      // g_print ("state: UNLINKED \n");
      break;
    case PA_SOURCE_INVALID_STATE:
      description.state_ = "n/a";
      // g_print ("state: n/a \n");
      break;
    case PA_SOURCE_RUNNING:
      description.state_ = "RUNNING";
      // g_print ("state: RUNNING \n");
      break;
    case PA_SOURCE_IDLE:
      description.state_ = "IDLE";
      // g_print ("state: IDLE \n");
      break;
    case PA_SOURCE_SUSPENDED:
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

  if (i->proplist && pa_proplist_contains(i->proplist, "device.bus_path")) {
    const void* bus_path = nullptr;
    size_t size;
    pa_proplist_get(i->proplist, "device.bus_path", &bus_path, &size);
    if (size) description.bus_path_ = static_cast<const char*>(bus_path);
  }

  gchar* rate = g_strdup_printf("%u", i->sample_spec.rate);
  description.sample_rate_ = rate;
  g_free(rate);
  gchar* channels = g_strdup_printf("%u", i->sample_spec.channels);
  description.channels_ = channels;
  g_free(channels);
  if (i->ports) {
    pa_source_port_info** p;
    for (p = i->ports; *p; p++) {
      description.ports_.push_back(std::make_pair((*p)->name, (*p)->description));
    }
  }
  if (i->active_port) {
    description.active_port_ = i->active_port->description;
  } else
    description.active_port_ = "n/a";
  context->capture_devices_.push_back(description);
  context->update_capture_device();
}

void PulseSrc::make_device_description(pa_context* pulse_context) {
  if (!capture_devices_.empty()) capture_devices_.clear();
  pa_operation_unref(
      pa_context_get_source_info_list(pulse_context, get_source_info_callback, this));
}

void PulseSrc::on_pa_event_callback(pa_context* pulse_context,
                                    pa_subscription_event_type_t pulse_event_type,
                                    uint32_t /*index */,
                                    void* user_data) {
  PulseSrc* context = static_cast<PulseSrc*>(user_data);
  if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SOURCE) {
    if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_NEW) {
      context->make_device_description(pulse_context);
      return;
    }
  }
  if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SOURCE) {
    if ((pulse_event_type & PA_SUBSCRIPTION_EVENT_TYPE_MASK) == PA_SUBSCRIPTION_EVENT_REMOVE) {
      context->make_device_description(pulse_context);
      return;
    }
  }
}

void PulseSrc::update_capture_device() {
  std::vector<std::string> names;
  std::vector<std::string> nicks;
  for (auto& it : capture_devices_) {
    names.push_back(it.description_);
    nicks.push_back(it.name_);
  }
  devices_ = Selection<>(std::make_pair(names, nicks), 0);
}

bool PulseSrc::start() {
  g_object_set(G_OBJECT(pulsesrc_.get_raw()),
               "device",
               capture_devices_.at(devices_.get_current_index()).name_.c_str(),
               nullptr);
  shm_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmsink_.get_raw(),
      [this](const std::string& caps) {
        this->graft_tree(
            ".shmdata.writer." + shmpath_,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.writer." + shmpath_));
  gst_bin_add_many(
      GST_BIN(gst_pipeline_->get_pipeline()), pulsesrc_.get_raw(), shmsink_.get_raw(), nullptr);
  gst_element_link_many(pulsesrc_.get_raw(), shmsink_.get_raw(), nullptr);
  gst_pipeline_->play(true);
  pmanage<MPtr(&PContainer::disable)>(devices_id_, StartableQuiddity::disabledWhenStartedMsg);
  return true;
}

bool PulseSrc::stop() {
  shm_sub_.reset(nullptr);
  prune_tree(".shmdata.writer." + shmpath_);
  pmanage<MPtr(&PContainer::remove)>(volume_id_);
  volume_id_ = 0;
  pmanage<MPtr(&PContainer::remove)>(mute_id_);
  mute_id_ = 0;
  if (!remake_elements()) return false;
  volume_id_ = pmanage<MPtr(&PContainer::push)>(
      "volume", GPropToProp::to_prop(G_OBJECT(pulsesrc_.get_raw()), "volume"));
  mute_id_ = pmanage<MPtr(&PContainer::push)>(
      "mute", GPropToProp::to_prop(G_OBJECT(pulsesrc_.get_raw()), "mute"));
  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  pmanage<MPtr(&PContainer::enable)>(devices_id_);
  return true;
}

InfoTree::ptr PulseSrc::on_saving() {
  auto res = InfoTree::make();
  DeviceDescription& desc = capture_devices_[devices_.get_current_index()];
  res->graft(".device_id", InfoTree::make(desc.name_));
  res->graft(".bus_path", InfoTree::make(desc.bus_path_));
  std::string save_mode = save_device_enum_.get_current_index() == 0 ? "port" : "device";
  res->graft(".save_by", InfoTree::make(save_mode));
  return res;
}

void PulseSrc::on_loading(InfoTree::ptr&& tree) {
  if (!tree || tree->empty()) {
    warning("loading deprecated pulsesrc device save: devices may swap when reloading");
    return;
  }

  std::string device_id = tree->branch_read_data<std::string>(".device_id");
  std::string bus_path = tree->branch_read_data<std::string>(".bus_path");
  std::string save_mode = tree->branch_read_data<std::string>(".save_by");

  if (save_mode == "port") {
    auto it = std::find_if(
        capture_devices_.begin(), capture_devices_.end(), [&](const DeviceDescription& capt) {
          return capt.bus_path_ == bus_path;
        });
    if (capture_devices_.end() == it) {
      warning("pulsesrc device not found at this port %", bus_path);
      message(
          "Audio capture device not found on its saved port when loading saved scenario, "
          "defaulting to first available device");
    } else {
      pmanage<MPtr(&PContainer::set<IndexOrName>)>(devices_id_,
                                                   IndexOrName(it - capture_devices_.begin()));
    }
  } else {  // save by device
    auto it = std::find_if(capture_devices_.begin(),
                           capture_devices_.end(),
                           [&](const DeviceDescription& capt) { return capt.name_ == device_id; });
    if (capture_devices_.end() == it) {
      warning("pulsesrc device not found (%)", device_id);
      message(
          "Saved audio capture not found when loading saved scenario, defaulting to first "
          "available device");
    } else {
      pmanage<MPtr(&PContainer::set<IndexOrName>)>(devices_id_,
                                                   IndexOrName(it - capture_devices_.begin()));
    }
  }

  // Locking the device selection while loading.
  is_loading_ = true;
}

void PulseSrc::on_loaded() { is_loading_ = true; }

}  // namespace switcher
