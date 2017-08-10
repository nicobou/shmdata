/*
 * This file is part of switcher-pulse.
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

#ifndef __SWITCHER_PULSE_SRC_H__
#define __SWITCHER_PULSE_SRC_H__

#include <pulse/glib-mainloop.h>
#include <pulse/pulseaudio.h>
#include <condition_variable>
#include <mutex>
#include "switcher/glibmainloop.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/unique-gst-element.hpp"

namespace switcher {
class PulseSrc : public Quiddity, public StartableQuiddity {
 public:
  PulseSrc(QuiddityConfiguration&&);
  ~PulseSrc();
  PulseSrc(const PulseSrc&) = delete;
  PulseSrc& operator=(const PulseSrc&) = delete;

 private:
  typedef struct {
    std::string name_{};
    std::string description_{};
    std::string state_{};
    std::string sample_format_{};
    std::string sample_rate_{};
    std::string channels_{};
    std::vector<std::pair<std::string /*port*/, std::string /*description*/>> ports_{};
    std::string active_port_{};
    std::string bus_path_{};
  } DeviceDescription;

  std::unique_ptr<GlibMainLoop> mainloop_;
  std::string shmpath_{};
  UGstElem pulsesrc_{"pulsesrc"};
  UGstElem shmsink_{"shmdatasink"};
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};
  // pulse devices
  bool connected_to_pulse_{false};
  std::mutex devices_mutex_{};
  std::condition_variable devices_cond_{};
  // property:
  // device enum members
  Selection<> devices_{{"none"}, 0};
  PContainer::prop_id_t devices_id_{0};
  PContainer::prop_id_t volume_id_{0};
  PContainer::prop_id_t mute_id_{0};
  Selection<> save_device_enum_{{"port", "device"}, 0};
  bool is_loading_{false};

  // pulse_audio
  pa_glib_mainloop* pa_glib_mainloop_{nullptr};
  pa_mainloop_api* pa_mainloop_api_{nullptr};
  pa_context* pa_context_{nullptr};
  char* server_{nullptr};
  std::vector<DeviceDescription> capture_devices_{};
  // quit
  std::mutex quit_mutex_{};
  std::condition_variable quit_cond_{};

  bool start() final;
  bool stop() final;
  InfoTree::ptr on_saving() final;
  void on_loading(InfoTree::ptr&& tree) final;
  void on_loaded() final;

  bool remake_elements();
  static gboolean async_get_pulse_devices(void* user_data);
  void update_capture_device();
  void make_device_description(pa_context* pulse_context);
  // device enum and select
  static void pa_context_state_callback(pa_context* c, void* userdata);
  static void get_source_info_callback(pa_context* c,
                                       const pa_source_info* i,
                                       int is_last,
                                       void* userdata);
  static void on_pa_event_callback(pa_context* c,
                                   pa_subscription_event_type_t t,
                                   uint32_t idx,
                                   void* userdata);
  static gboolean quit_pulse(void* user_data);
};

SWITCHER_DECLARE_PLUGIN(PulseSrc);
}  // namespace switcher
#endif
