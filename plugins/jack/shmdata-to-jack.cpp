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

#include "./shmdata-to-jack.hpp"
#include "./audio-resampler.hpp"
#include "switcher/gst/utils.hpp"
#include "switcher/quiddity/container.hpp"
#include "switcher/quiddity/property/gprop-to-prop.hpp"
#include "switcher/utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToJack,
                                     "jacksink",
                                     "Audio Display (Jack)",
                                     "Audio display",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string ShmdataToJack::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "audio",
      "description": "Audio stream to expose to Jack",
      "can_do": ["audio/x-raw"]
    }
  ]
}
)");

ShmdataToJack::ShmdataToJack(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t sfid) { return on_shmdata_disconnect(); }}),
      client_name_(get_nickname()),
      server_name_(conf.tree_config_->branch_has_data("server_name")
                       ? conf.tree_config_->branch_get_value("server_name").copy_as<std::string>()
                       : "default"),
      client_name_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "client_name",
          [this](const std::string& val) {
            client_name_ = val;
            return true;
          },
          [this]() { return client_name_; },
          "Client Name",
          "The jack client name",
          client_name_)),
      server_name_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "server_name",
          [this](const std::string& val) {
            server_name_ = val;
            return true;
          },
          [this]() { return server_name_; },
          "Server Name",
          "The jack server name",
          server_name_)),
      connect_to_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "connect_to",
          [this](const std::string& val) {
            connect_to_ = val;
            return true;
          },
          [this]() { return connect_to_; },
          "Connect To",
          "Which client to connect to",
          connect_to_)),
      auto_connect_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "auto_connect",
          [this](const bool val) {
            auto_connect_ = val;
            if (auto_connect_) {
              pmanage<MPtr(&property::PBag::enable)>(connect_to_id_);
              pmanage<MPtr(&property::PBag::enable)>(index_id_);
              pmanage<MPtr(&property::PBag::enable)>(connect_all_to_first_id_);
              pmanage<MPtr(&property::PBag::enable)>(connect_only_first_id_);
            } else {
              static const std::string why_disabled =
                  "this property is available only when auto connect is enabled";
              pmanage<MPtr(&property::PBag::disable)>(connect_to_id_, why_disabled);
              pmanage<MPtr(&property::PBag::disable)>(index_id_, why_disabled);
              pmanage<MPtr(&property::PBag::disable)>(connect_all_to_first_id_, why_disabled);
              pmanage<MPtr(&property::PBag::disable)>(connect_only_first_id_, why_disabled);
            }
            return true;
          },
          [this]() { return auto_connect_; },
          "Auto Connect",
          "Auto Connect to another client",
          auto_connect_)),
      connect_all_to_first_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "connect_all_to_first",
          [this](const bool val) {
            connect_all_to_first_ = val;
            return true;
          },
          [this]() { return connect_all_to_first_; },
          "Many Channels To One",
          "Connect all channels to the first",
          connect_all_to_first_)),
      connect_only_first_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "connect_only_first",
          [this](const bool val) {
            connect_only_first_ = val;
            return true;
          },
          [this]() { return connect_only_first_; },
          "Connect only first channel",
          "Connect only first channel",
          connect_only_first_)),
      do_format_conversion_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "do_format_conversion",
          [this](const bool val) {
            do_format_conversion_ = val;
            return true;
          },
          [this]() { return do_format_conversion_; },
          "Do sample conversion to float (F32LE)",
          "Do sample conversion to float (F32LE)",
          do_format_conversion_)),
      do_rate_conversion_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "do_rate_conversion",
          [this](const bool val) {
            do_rate_conversion_ = val;
            return true;
          },
          [this]() { return do_format_conversion_; },
          "Convert to Jack rate",
          "Convert to Jack rate",
          do_rate_conversion_)),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)) {
  unsigned int max_number_of_channels = kMaxNumberOfChannels;
  if (config<MPtr(&InfoTree::branch_has_data)>("max_number_of_channels"))
    max_number_of_channels =
        config<MPtr(&InfoTree::branch_get_value)>("max_number_of_channels").copy_as<unsigned int>();

  index_id_ = pmanage<MPtr(&property::PBag::make_int)>(
      "index",
      [this](const int& val) {
        index_ = val;
        return true;
      },
      [this]() { return index_; },
      "Channel",
      "Start connecting to other client from this channel index",
      index_,
      0,
      max_number_of_channels);
}

int ShmdataToJack::jack_process(jack_nframes_t nframes, void* arg) {
  auto context = static_cast<ShmdataToJack*>(arg);
  if (!context->is_constructed_) return 0;
  {
    std::lock_guard<std::mutex> lock(context->port_to_connect_in_jack_process_mutex_);
    for (auto& it : context->port_to_connect_in_jack_process_)
      jack_connect(context->jack_client_->get_raw(), it.first.c_str(), it.second.c_str());
    context->port_to_connect_in_jack_process_.clear();
  }
  {
    std::unique_lock<std::mutex> lock(context->output_ports_mutex_, std::defer_lock);
    if (lock.try_lock()) {
      auto write_zero = false;
      auto num_channels = context->output_ports_.size();
      if (context->ring_buffers_.empty() ||
          (num_channels > 0 && context->ring_buffers_[0].get_usage() < nframes)) {
        write_zero = true;
      }
      for (unsigned int i = 0; i < context->output_ports_.size(); ++i) {
        jack_sample_t* buf = static_cast<jack_sample_t*>(
            jack_port_get_buffer(context->output_ports_[i].get_raw(), nframes));
        if (!buf) return 0;
        if (!write_zero) {
          context->ring_buffers_[i].pop_samples(static_cast<std::size_t>(nframes), buf);
        } else {
          for (unsigned int j = 0; j < nframes; ++j) buf[j] = 0;
        }
      }
    }  // locked
  }    // releasing lock
  return 0;
}

void ShmdataToJack::on_xrun(uint num_of_missed_samples) {
  if (!is_constructed_) return;
  info("jack xrun (delay of % samples)", std::to_string(num_of_missed_samples));
  jack_nframes_t jack_buffer_size = jack_client_->get_buffer_size();
  for (auto& it : ring_buffers_) {
    // this is safe since on_xrun is called right before jack_process,
    // on the same thread
    it.shrink_to(static_cast<std::size_t>(jack_buffer_size * 1.5));
  }
}

void ShmdataToJack::on_handoff_cb(GstElement* /*object*/,
                                  GstBuffer* buf,
                                  GstPad* pad,
                                  gpointer user_data) {
  ShmdataToJack* context = static_cast<ShmdataToJack*>(user_data);
  auto current_time = jack_frame_time(context->jack_client_->get_raw());
  GstCaps* caps = gst_pad_get_current_caps(pad);
  if (nullptr == caps) return;
  On_scope_exit { gst_caps_unref(caps); };
  // gchar *string_caps = gst_caps_to_string(caps);
  // On_scope_exit {if (nullptr != string_caps) g_free(string_caps);};
  // debug("on handoff, negotiated caps is %", std::string(string_caps));
  const GValue* val = gst_structure_get_value(gst_caps_get_structure(caps, 0), "channels");
  const int channels = g_value_get_int(val);
  if (channels != context->channels_) {
    context->on_channel_update(channels);
    if (channels > 0) context->connect_ports();
  }
  // getting buffer infomation:
  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
    context->warning("gst_buffer_map failed: canceling audio buffer access");
    return;
  }
  On_scope_exit { gst_buffer_unmap(buf, &map); };
  jack_nframes_t duration = map.size / (4 * channels);
  // setting the smoothing value affecting (20 sec)
  context->drift_observer_.set_smoothing_factor(
      static_cast<double>(duration) /
      (20.0 * static_cast<double>(context->jack_client_->get_sample_rate())));
  std::size_t new_size = static_cast<std::size_t>(
      context->drift_observer_.set_current_time_info(current_time, duration));
  --context->debug_buffer_usage_;
  if (0 == context->debug_buffer_usage_) {
    context->debug("buffer load is %, ratio is %",
                   std::to_string(context->ring_buffers_[0].get_usage()),
                   std::to_string(context->drift_observer_.get_ratio()));
    context->debug_buffer_usage_ = 1000;
  }
  jack_sample_t* tmp_buf = (jack_sample_t*)map.data;
  for (int i = 0; i < channels; ++i) {
    AudioResampler<jack_sample_t> resample(duration, new_size, tmp_buf, i, channels);
    auto emplaced = context->ring_buffers_[i].put_samples(new_size, [&resample]() {
      // return resample.zero_pole_get_next_sample();
      return resample.linear_get_next_sample();
    });
    if (emplaced != new_size)
      context->warning("overflow of % samples", std::to_string(new_size - emplaced));
  }
}

bool ShmdataToJack::make_elements() {
  GError* error = nullptr;
  auto src = std::string("shmdatasrc ");
  if (do_format_conversion_) src += " ! audioconvert ";
  if (do_rate_conversion_) src += " ! audioresample ";
  if (do_format_conversion_ && do_rate_conversion_) src += " ! audioconvert ";
  std::string description(src +
                          " ! capsfilter caps=\"audio/x-raw, format=(string)F32LE, "
                          "layout=(string)interleaved, rate=" +
                          std::to_string(jack_client_->get_sample_rate()) + "\" !" +
                          " fakesink silent=true signal-handoffs=true sync=false");
  GstElement* jacksink = gst_parse_bin_from_description(description.c_str(), TRUE, &error);
  if (error != nullptr) {
    warning("error making gst elements in jacksink: %", std::string(error->message));
    g_error_free(error);
    return false;
  }
  GstElement* shmdatasrc =
      gst::utils::get_first_element_from_factory_name(GST_BIN(jacksink), "shmdatasrc");
  GstElement* fakesink =
      gst::utils::get_first_element_from_factory_name(GST_BIN(jacksink), "fakesink");
  handoff_handler_ = g_signal_connect(fakesink, "handoff", (GCallback)on_handoff_cb, this);
  if (nullptr != audiobin_) gst::utils::clean_element(audiobin_);
  shmdatasrc_ = shmdatasrc;
  audiobin_ = jacksink;
  return true;
}

bool ShmdataToJack::start() {
  if (shmpath_.empty()) {
    warning("cannot start, no shmdata to connect with");
    return false;
  }

  // Create JACK Client
  channels_ = 0;  // Channels will be dynamically discovered
  jack_client_ = std::make_unique<JackClient>(
      client_name_,
      server_name_.empty() ? "default" : server_name_,
      &ShmdataToJack::jack_process,
      this,
      [this](uint n) { on_xrun(n); },
      [this](jack_port_t* port) { on_port(port); },
      [this]() {
        if (!is_constructed_) return;
        auto thread = std::thread([this]() {
          if (!qcontainer_->remove(get_id()))
            warning("% did not self destruct after jack shutdown", get_nickname());
        });
        thread.detach();
      });
  if (!*jack_client_.get()) {
    message("ERROR: JackClient cannot be instantiated (is jack server running?)");
    is_valid_ = false;
    return false;
  }
  is_constructed_ = true;  // needed because of a cross reference between JackClient and JackPort
  client_name_ = jack_client_->get_name();

  // Create and start gst pipeline
  if (!gst_pipeline_) gst_pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);
  if (!make_elements()) {
    is_valid_ = false;
    return false;
  }
  g_object_set(G_OBJECT(shmdatasrc_), "socket-path", shmpath_.c_str(), nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), audiobin_);
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  gst_pipeline_->play(true);

  // Create shmdata follower
  shm_sub_ = std::make_unique<shmdata::GstTreeUpdater>(
      this, shmdatasrc_, shmpath_, shmdata::GstTreeUpdater::Direction::reader);

  // Disable properties
  pmanage<MPtr(&property::PBag::disable)>(server_name_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(client_name_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(auto_connect_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(connect_to_id_, property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(index_id_, property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(connect_all_to_first_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(connect_only_first_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(do_format_conversion_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(do_rate_conversion_id_,
                                          property::PBag::disabledWhenConnectedMsg);

  return true;
}

bool ShmdataToJack::stop() {
  // Stop processing
  disconnect_ports();
  gst_pipeline_->play(false);

  // Reset gst-related stuff
  shm_sub_.reset();
  gst_pipeline_.reset();
  shmpath_.clear();

  // Clean JACK-related stuff
  clean_output_ports();  // Must clean ports before unreffing jack_client
  jack_client_.reset();
  is_constructed_ = false;

  // Enable properties
  pmanage<MPtr(&property::PBag::enable)>(server_name_id_);
  pmanage<MPtr(&property::PBag::enable)>(client_name_id_);
  pmanage<MPtr(&property::PBag::enable)>(auto_connect_id_);
  pmanage<MPtr(&property::PBag::enable)>(connect_to_id_);
  pmanage<MPtr(&property::PBag::enable)>(index_id_);
  pmanage<MPtr(&property::PBag::enable)>(connect_all_to_first_id_);
  pmanage<MPtr(&property::PBag::enable)>(connect_only_first_id_);
  pmanage<MPtr(&property::PBag::enable)>(do_format_conversion_id_);
  pmanage<MPtr(&property::PBag::enable)>(do_rate_conversion_id_);

  return true;
}

bool ShmdataToJack::on_shmdata_disconnect() {
  if (shmpath_.empty()) {
    return false;
  }
  return stop();
}

bool ShmdataToJack::on_shmdata_connect(const std::string& shmpath) {
  if (shmpath.empty()) {
    error("shmpath must not be empty");
    return false;
  }
  if (!shmpath_.empty()) {
    stop();
  }
  shmpath_ = shmpath;
  return start();
}

void ShmdataToJack::on_channel_update(unsigned int channels) {
  channels_ = channels;
  // thread safe operations on output ports
  {
    std::unique_lock<std::mutex> lock(output_ports_mutex_);
    // unregistering previous ports
    output_ports_.clear();
    // replacing with new ports
    for (unsigned int i = 0; i < channels; ++i) output_ports_.emplace_back(*jack_client_, i);
    update_ports_to_connect();
    // replacing ring buffers
    std::vector<AudioRingBuffer<jack_sample_t>> tmp(channels);
    std::swap(ring_buffers_, tmp);
  }  // unlocking output_ports_
}

void ShmdataToJack::update_ports_to_connect() {
  std::lock_guard<std::mutex> lock(ports_to_connect_mutex_);
  ports_to_connect_.clear();

  for (unsigned int i = index_; i < index_ + channels_; ++i) {
    char buff[128];
    std::snprintf(buff, sizeof(buff), connect_to_.c_str(), i);
    ports_to_connect_.emplace_back(buff);
  }
}

void ShmdataToJack::connect_ports() {
  if (output_ports_.empty()) return;
  if (!auto_connect_) return;

  {
    std::lock_guard<std::mutex> lock(output_ports_mutex_);
    if (ports_to_connect_.size() != output_ports_.size()) {
      warning("Port number mismatch in shmdata to jack autoconnect, should not happen.");
      return;
    }

    for (unsigned int i = 0; i < (connect_only_first_ ? 1 : output_ports_.size()); ++i) {
      unsigned int dest_port_index = connect_all_to_first_ ? 0 : i;
      debug("Connecting % to %",
            std::string(client_name_ + ":" + output_ports_[i].get_name()),
            ports_to_connect_[dest_port_index]);
      jack_connect(jack_client_->get_raw(),
                   std::string(client_name_ + ":" + output_ports_[i].get_name()).c_str(),
                   ports_to_connect_[dest_port_index].c_str());
    }
  }
}

void ShmdataToJack::disconnect_ports() {
  std::lock_guard<std::mutex> lock(output_ports_mutex_);
  for (auto& it : output_ports_) jack_port_disconnect(jack_client_->get_raw(), it.get_raw());
}

void ShmdataToJack::clean_output_ports() {
  std::lock_guard<std::mutex> lock(output_ports_mutex_);
  output_ports_.clear();
}

void ShmdataToJack::on_port(jack_port_t* port) {
  if (!is_constructed_) return;
  std::lock_guard<std::mutex> lock(ports_to_connect_mutex_);
  int flags = jack_port_flags(port);
  if (!(flags & JackPortIsInput)) return;
  auto it = std::find(ports_to_connect_.begin(), ports_to_connect_.end(), jack_port_name(port));
  if (ports_to_connect_.end() == it) return;
  {
    std::lock_guard<std::mutex> lock(port_to_connect_in_jack_process_mutex_);
    port_to_connect_in_jack_process_.push_back(std::make_pair(
        std::string(client_name_ + ":" + output_ports_[it - ports_to_connect_.begin()].get_name()),
        *it));
  }
}

}  // namespace quiddities
}  // namespace switcher
