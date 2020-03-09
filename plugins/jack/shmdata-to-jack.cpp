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
                                     "audio",
                                     "reader/device",
                                     "Audio display",
                                     "LGPL",
                                     "Nicolas Bouillot");

ShmdataToJack::ShmdataToJack(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      connect_to_id_(pmanage<MPtr(&property::PBag::make_string)>("connect_to",
                                                                 [this](const std::string& val) {
                                                                   connect_to_ = val;
                                                                   update_port_to_connect();
                                                                   return true;
                                                                 },
                                                                 [this]() { return connect_to_; },
                                                                 "Connect To",
                                                                 "Which client to connect to",
                                                                 connect_to_)),
      auto_connect_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "auto_connect",
          [this](const bool& val) {
            auto_connect_ = val;
            update_port_to_connect();
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
      connect_all_to_first_id_(
          pmanage<MPtr(&property::PBag::make_bool)>("connect_all_to_first",
                                                    [this](const bool& val) {
                                                      connect_all_to_first_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return connect_all_to_first_; },
                                                    "Many Channels To One",
                                                    "Connect all channels to the first",
                                                    connect_all_to_first_)),
      connect_only_first_id_(
          pmanage<MPtr(&property::PBag::make_bool)>("connect_only_first",
                                                    [this](const bool& val) {
                                                      connect_only_first_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return connect_only_first_; },
                                                    "Connect only first channel",
                                                    "Connect only first channel",
                                                    connect_only_first_)),
      do_format_conversion_id_(
          pmanage<MPtr(&property::PBag::make_bool)>("do_format_conversion",
                                                    [this](const bool& val) {
                                                      do_format_conversion_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return do_format_conversion_; },
                                                    "Do sample conversion to float (F32LE)",
                                                    "Do sample conversion to float (F32LE)",
                                                    do_format_conversion_)),
      do_rate_conversion_id_(
          pmanage<MPtr(&property::PBag::make_bool)>("do_rate_conversion",
                                                    [this](const bool& val) {
                                                      do_rate_conversion_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return do_format_conversion_; },
                                                    "Convert to Jack rate",
                                                    "Convert to Jack rate",
                                                    do_rate_conversion_)),
      shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)),
      jack_client_(get_name(),
                   conf.tree_config_->branch_has_data("server_name")
                       ? conf.tree_config_->branch_get_value("server_name").copy_as<std::string>()
                       : std::string(),
                   &ShmdataToJack::jack_process,
                   this,
                   [this](uint n) { on_xrun(n); },
                   [this](jack_port_t* port) { on_port(port); },
                   [this]() {
                     if (!is_constructed_) return;
                     auto thread = std::thread([this]() {
                       if (!qcontainer_->remove(qcontainer_->get_id(get_name())))
                         warning("% did not self destruct after jack shutdown", get_name());
                     });
                     thread.detach();
                   }) {
  // is_constructed_ is needed because of a cross reference among JackClient and JackPort
  is_constructed_ = true;
  if (!jack_client_) {
    message("ERROR:JackClient cannot be instanciated (is jack server running?)");
    is_valid_ = false;
    return;
  }
  if (!make_elements()) {
    is_valid_ = false;
    return;
  }
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
  unsigned int max_number_of_channels = kMaxNumberOfChannels;
  if (config<MPtr(&InfoTree::branch_has_data)>("max_number_of_channels"))
    max_number_of_channels =
        config<MPtr(&InfoTree::branch_get_value)>("max_number_of_channels").copy_as<unsigned int>();
  index_id_ = pmanage<MPtr(&property::PBag::make_int)>(
      "index",
      [this](const int& val) {
        index_ = val;
        update_port_to_connect();
        return true;
      },
      [this]() { return index_; },
      "Channel",
      "Start connecting to other client from this channel index",
      index_,
      0,
      max_number_of_channels);
  update_port_to_connect();
}

int ShmdataToJack::jack_process(jack_nframes_t nframes, void* arg) {
  auto context = static_cast<ShmdataToJack*>(arg);
  if (!context->is_constructed_) return 0;
  {
    std::lock_guard<std::mutex> lock(context->port_to_connect_in_jack_process_mutex_);
    for (auto& it : context->port_to_connect_in_jack_process_)
      jack_connect(context->jack_client_.get_raw(), it.first.c_str(), it.second.c_str());
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
        jack_sample_t* buf =
            (jack_sample_t*)jack_port_get_buffer(context->output_ports_[i].get_raw(), nframes);
        if (!buf) return 0;
        if (!write_zero) {
          context->ring_buffers_[i].pop_samples((std::size_t)nframes, buf);
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
  warning("jack xrun (delay of % samples)", std::to_string(num_of_missed_samples));
  jack_nframes_t jack_buffer_size = jack_client_.get_buffer_size();
  for (auto& it : ring_buffers_) {
    // this is safe since on_xrun is called right before jack_process,
    // on the same thread
    it.shrink_to((std::size_t)jack_buffer_size * 1.5);
  }
}

void ShmdataToJack::on_handoff_cb(GstElement* /*object*/,
                                  GstBuffer* buf,
                                  GstPad* pad,
                                  gpointer user_data) {
  ShmdataToJack* context = static_cast<ShmdataToJack*>(user_data);
  auto current_time = jack_frame_time(context->jack_client_.get_raw());
  GstCaps* caps = gst_pad_get_current_caps(pad);
  if (nullptr == caps) return;
  On_scope_exit { gst_caps_unref(caps); };
  // gchar *string_caps = gst_caps_to_string(caps);
  // On_scope_exit {if (nullptr != string_caps) g_free(string_caps);};
  // debug("on handoff, negotiated caps is %", std::string(string_caps));
  const GValue* val = gst_structure_get_value(gst_caps_get_structure(caps, 0), "channels");
  const int channels = g_value_get_int(val);
  context->check_output_ports(channels);
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
      (double)duration / (20.0 * (double)context->jack_client_.get_sample_rate()));
  std::size_t new_size =
      (std::size_t)context->drift_observer_.set_current_time_info(current_time, duration);
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

void ShmdataToJack::check_output_ports(unsigned int channels) {
  if (channels == channels_) return;
  channels_ = channels;
  // thread safe operations on output ports
  {
    std::unique_lock<std::mutex> lock(output_ports_mutex_);
    // unregistering previous ports
    output_ports_.clear();
    // replacing with new ports
    for (unsigned int i = 0; i < channels; ++i) output_ports_.emplace_back(jack_client_, i);
    if (channels > 0) connect_ports();
    // replacing ring buffers
    std::vector<AudioRingBuffer<jack_sample_t>> tmp(channels);
    std::swap(ring_buffers_, tmp);
  }  // unlocking output_ports_
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
                          std::to_string(jack_client_.get_sample_rate()) + "\" !" +
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
  fakesink_ = fakesink;
  return true;
}

bool ShmdataToJack::start() {
  if (shmpath_.empty()) {
    warning("cannot start, no shmdata to connect with");
    return false;
  }
  g_object_set(G_OBJECT(shmdatasrc_), "socket-path", shmpath_.c_str(), nullptr);
  shm_sub_ = std::make_unique<shmdata::GstTreeUpdater>(
      this, shmdatasrc_, shmpath_, shmdata::GstTreeUpdater::Direction::reader);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), audiobin_);
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  gst_pipeline_->play(true);
  pmanage<MPtr(&property::PBag::disable)>(auto_connect_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(connect_to_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(index_id_, shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(connect_all_to_first_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(connect_only_first_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  connect_ports();
  return true;
}

bool ShmdataToJack::stop() {
  shm_sub_.reset();
  disconnect_ports();
  {
    On_scope_exit { gst_pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr); };
    if (!make_elements()) return false;
  }
  pmanage<MPtr(&property::PBag::enable)>(auto_connect_id_);
  pmanage<MPtr(&property::PBag::enable)>(connect_to_id_);
  pmanage<MPtr(&property::PBag::enable)>(index_id_);
  pmanage<MPtr(&property::PBag::enable)>(connect_all_to_first_id_);
  pmanage<MPtr(&property::PBag::enable)>(connect_only_first_id_);
  return true;
}

bool ShmdataToJack::on_shmdata_disconnect() { return stop(); }

bool ShmdataToJack::on_shmdata_connect(const std::string& shmpath) {
  shmpath_ = shmpath;
  stop();
  return start();
}

bool ShmdataToJack::can_sink_caps(const std::string& caps) {
  return gst::utils::can_sink_caps("audioconvert", caps);
}

void ShmdataToJack::update_port_to_connect() {
  std::lock_guard<std::mutex> lock(ports_to_connect_mutex_);
  ports_to_connect_.clear();

  if (!auto_connect_) {
    warning("Auto-connect for jack is disabled.");
    return;
  }

  for (unsigned int i = index_; i < index_ + output_ports_.size(); ++i) {
    char buff[128];
    std::snprintf(buff, sizeof(buff), connect_to_.c_str(), i);
    ports_to_connect_.emplace_back(buff);
  }
}

void ShmdataToJack::connect_ports() {
  update_port_to_connect();

  if (0 == output_ports_.size()) return;
  if (!auto_connect_) return;

  std::lock_guard<std::mutex> lock(ports_to_connect_mutex_);
  if (ports_to_connect_.size() != output_ports_.size()) {
    warning(
        "Port number mismatch in shmdata to jack autoconnect, should not "
        "happen.");
    return;
  }

  for (unsigned int i = 0; i < (connect_only_first_ ? 1 : output_ports_.size()); ++i) {
    unsigned int dest_port_index = connect_all_to_first_ ? 0 : i;
    jack_connect(jack_client_.get_raw(),
                 std::string(jack_client_.get_name() + ":" + output_ports_[i].get_name()).c_str(),
                 ports_to_connect_[dest_port_index].c_str());
  }
}

void ShmdataToJack::disconnect_ports() {
  for (auto& it : output_ports_) jack_port_disconnect(jack_client_.get_raw(), it.get_raw());
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
    port_to_connect_in_jack_process_.push_back(
        std::make_pair(std::string(jack_client_.get_name() + ":" +
                                   output_ports_[it - ports_to_connect_.begin()].get_name()),
                       *it));
  }
}

}  // namespace quiddities
}  // namespace switcher
