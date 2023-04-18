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

#include <pulse/mainloop.h>
#include <pulse/pulseaudio.h>

#include "switcher/gst/utils.hpp"
#include "switcher/quiddity/property/gprop-to-prop.hpp"
#include "switcher/utils/scope-exit.hpp"

using namespace std::chrono_literals;

namespace switcher {
namespace quiddities {

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSink,
                                     "pulsesink",
                                     "Audio Display (Pulse)",
                                     "Inspecting Devices And Playing Audio To Outputs",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string PulseSink::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "audio",
      "description": "Audio stream",
      "can_do": ["audio/x-raw"]
    }
  ]
}
)");

PulseSink::PulseSink(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t) { return on_shmdata_disconnect(); }}),
      mainloop_(std::make_unique<gst::GlibMainLoop>()) {
  std::unique_lock<std::mutex> lock(devices_mutex_);
  gst::utils::g_idle_add_full_with_context(mainloop_->get_main_context(),
                                           G_PRIORITY_DEFAULT_IDLE,
                                           async_get_pulse_devices,
                                           this,
                                           nullptr);
  devices_cond_.wait_for(lock, 1s);
  if (!connected_to_pulse_) {
    sw_info("Not connected to pulse, cannot initialize.");
    is_valid_ = false;
    return;
  }
}

PulseSink::~PulseSink() {
  GMainContext* main_context = mainloop_->get_main_context();
  if (nullptr != main_context && connected_to_pulse_) {
    std::unique_lock<std::mutex> lock(quit_mutex_);
    gst::utils::g_idle_add_full_with_context(
        main_context, G_PRIORITY_DEFAULT_IDLE, quit_pulse, this, nullptr);
    quit_cond_.wait_for(lock, 1s);
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
    context->sw_debug("PulseSink:: pa_context_new() failed.");
    return FALSE;
  }
  pa_context_set_state_callback(context->pa_context_, pa_context_state_callback, context);
  if (pa_context_connect(context->pa_context_, context->server_, (pa_context_flags_t)0, nullptr) <
      0) {
    context->sw_debug("pa_context_connect() failed: {}",
                      std::string(pa_strerror(pa_context_errno(context->pa_context_))));
    return FALSE;
  }
  context->connected_to_pulse_ = true;
  return FALSE;
}

bool PulseSink::remake_elements() {
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
      context->sw_debug("PulseSink Context error: {}",
                        std::string(pa_strerror(pa_context_errno(pulse_context))));
  }
}

void PulseSink::get_sink_info_callback(pa_context* pulse_context,
                                       const pa_sink_info* i,
                                       int is_last,
                                       void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  if (is_last < 0) {
    context->sw_debug("Failed to get sink information: {}",
                      std::string(pa_strerror(pa_context_errno(pulse_context))));
    return;
  }
  if (is_last) {
    pa_operation* operation = pa_context_drain(pulse_context, nullptr, nullptr);
    if (operation) pa_operation_unref(operation);
    context->update_output_device();
    context->devices_enum_id_ = context->pmanage<MPtr(&property::PBag::make_selection<>)>(
        "device",
        [context](const quiddity::property::IndexOrName& val) {
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
  devices_enum_ = property::Selection<>(std::make_pair(names, nicks), 0);
}

bool PulseSink::on_shmdata_disconnect() {
  pmanage<MPtr(&property::PBag::enable)>(devices_enum_id_);
  shmf_.reset();
  return remake_elements();
}

bool PulseSink::on_shmdata_connect(const std::string& shmpath) {
  pmanage<MPtr(&property::PBag::disable)>(devices_enum_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  shmpath_ = shmpath;
  shmf_ = std::make_unique<shmdata::Follower>(
      this,
      shmpath,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      [this](const std::string& caps) { this->on_shmreader_connected(caps); },
      [this]() { this->on_shmreader_disconnected(); },
      shmdata::Stat::kDefaultUpdateInterval,
      shmdata::Follower::Direction::reader,
      true);

  return true;
}

void PulseSink::on_shmreader_data(void* raw_data, size_t data_size) {
  if (!pa_stream_is_on_) return;

  auto duration = data_size / (shmf_caps_.format_size_in_bytes() * shmf_caps_.channels());
  std::vector<float> float_buffer;
  float* data = static_cast<float*>(raw_data);
  if (!shmf_caps_.is_float()) {
    float_buffer.reserve(data_size / shmf_caps_.format_size_in_bytes());
    for (std::size_t i = 0; i < data_size / shmf_caps_.format_size_in_bytes(); ++i) {
      if (shmf_caps_.is_signed()) {
        auto raw_data_formated = static_cast<int16_t*>(raw_data);
        float_buffer.push_back(static_cast<float>(raw_data_formated[i]) / 65535.);
      } else {  // assumin unsigned
        float_buffer.emplace_back(static_cast<int8_t*>(raw_data)[i] / 255.);
      }
    }
    data = float_buffer.data();
  }
  drift_observer_.set_smoothing_factor(static_cast<double>(duration) /
                                       (20.0 * static_cast<double>(shmf_caps_.samplerate())));
  std::size_t new_size =
      static_cast<std::size_t>(drift_observer_.set_current_time_info(pa_ts_, duration));
  --debug_buffer_usage_;
  if (0 == debug_buffer_usage_) {
    sw_debug("buffer load is {}, ratio is {}",
             std::to_string(ring_buffers_[0].get_usage()),
             std::to_string(drift_observer_.get_ratio()));
    debug_buffer_usage_ = 1000;
  }
  // Smoothly reduce latency if the ring buffer contain more than 10ms of audio
  if (ring_buffers_[0].get_usage() > shmf_caps_.samplerate() * 0.01) {
    new_size *= 0.9999;
  }
  audio_resampler_->do_resample(duration, new_size, static_cast<const float*>(data));

  for (unsigned int i = 0; i < shmf_caps_.channels(); ++i) {
    std::size_t cur_pos = 0;
    auto emplaced = ring_buffers_[i].put_samples(new_size, [&]() {
      auto pos = cur_pos;
      ++cur_pos;
      return audio_resampler_->get_sample(pos, i);
    });

    if (emplaced != new_size)
      sw_warning("overflow of {} samples", std::to_string(new_size - emplaced));
  }
}

void PulseSink::on_shmreader_connected(const std::string& shmdata_caps) {
  shmf_caps_ = shmdata::caps::AudioCaps(shmdata_caps);
  sw_info("Pulsesink connected to a shmdata with following caps: {}", shmdata_caps);
  if (!shmf_caps_) {
    sw_warning("Pulsesink unsupported caps: {}", shmdata_caps);
    return;
  }

  // replacing ring buffers
  std::vector<utils::AudioRingBuffer<float>> tmp(shmf_caps_.channels());
  std::swap(ring_buffers_, tmp);
  // restarting resampler
  audio_resampler_ = std::make_unique<utils::AudioResampler<float>>(this, shmf_caps_.channels());
  // creating the pulseaudio stream
  gst::utils::g_idle_add_full_with_context(
      mainloop_->get_main_context(), G_PRIORITY_DEFAULT_IDLE, async_create_stream, this, nullptr);
}

void PulseSink::on_shmreader_disconnected() {
  // removing the pulseaudio stream
  gst::utils::g_idle_add_full_with_context(
      mainloop_->get_main_context(), G_PRIORITY_DEFAULT_IDLE, async_remove_stream, this, nullptr);
}

gboolean PulseSink::async_create_stream(void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  auto channels = context->shmf_caps_.channels();
  if (channels > 32) {  // 32 is the maximum number of channels with pulseaudio
    context->sw_error(
        "Cannot create pulseaudio streams with more than 32 channels (audio shmdata has "
        "{} channels)",
        context->shmf_caps_.channels());
    return FALSE;
  }

  pa_stream_flags_t flags = PA_STREAM_ADJUST_LATENCY;
  pa_buffer_attr attr;
  static const pa_sample_spec ss = {.format = PA_SAMPLE_FLOAT32LE,
                                    .rate = context->shmf_caps_.samplerate(),
                                    .channels = static_cast<unsigned char>(channels)};

  memset(&attr, 0, sizeof(attr));
  attr.maxlength = (uint32_t)-1;
  attr.tlength = (uint32_t)context->shmf_caps_.samplerate() * 4 /*float*/ *
                 context->shmf_caps_.channels() / 1000;
  attr.prebuf = (uint32_t)0;
  attr.minreq = (uint32_t)-1;
  attr.fragsize = (uint32_t)-1;

  context->pa_stream_ =
      pa_stream_new(context->pa_context_, context->get_nickname().c_str(), &ss, nullptr);
  if (context->pa_stream_ == nullptr) {
    context->sw_error("Pulseaudio streams failed to create");
    return FALSE;
  }

  if (pa_stream_connect_playback(context->pa_stream_, nullptr, &attr, flags, nullptr, nullptr) !=
      0) {
    context->sw_error("Pulseaudio failed to connect playback");
    return FALSE;
  }
  pa_stream_set_write_callback(context->pa_stream_, stream_write_cb, context);

  return FALSE;
}

gboolean PulseSink::async_remove_stream(void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);
  pa_stream_disconnect(context->pa_stream_);
  context->pa_stream_ = nullptr;
  context->pa_stream_is_on_ = false;
  return FALSE;
}

void PulseSink::stream_write_cb(pa_stream* p, size_t nbytes, void* user_data) {
  PulseSink* context = static_cast<PulseSink*>(user_data);

  context->pa_stream_is_on_ = true;

  for (;;) {
    void* data;

    if ((nbytes = pa_stream_writable_size(p)) == (size_t)-1) {
      context->sw_error("Pulseaudio stream not writable");
    }

    if (nbytes <= 0) break;

    size_t nframes = nbytes / (4 /*float32*/ * context->shmf_caps_.channels());
    context->pa_ts_ += nframes;
    auto write_zero = false;
    if (context->ring_buffers_.empty() ||
        (context->shmf_caps_.channels() > 0 && context->ring_buffers_[0].get_usage() < nframes)) {
      write_zero = true;
      context->sw_info(
          "pulsesink: ring buffer empty or not enough sample available for playback. "
          "Writing zeros.");
    }

    if (pa_stream_begin_write(p, &data, &nbytes) != 0) {
      context->sw_error("Pulseaudio failed to begin write");
    }
    for (unsigned int i = 0; i < context->shmf_caps_.channels(); ++i) {
      if (!write_zero) {
        context->ring_buffers_[i].pop_samples_as_channel(
            nframes, static_cast<float*>(data), i + 1, context->shmf_caps_.channels());
      } else {
        /* Just some silence */
        memset(data, 0, nbytes);
      }
    }

    if (pa_stream_write(p, data, nbytes, nullptr, 0, PA_SEEK_RELATIVE) != 0) {
      context->sw_error("Pulseaudio failed write into the stream");
    }
  }
}

}  // namespace quiddities
}  // namespace switcher
