/*
 * This file is part of switcher-curl.
 *
 * switcher-curl is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_HTTP_GET_SERVICE_H__
#define __SWITCHER_HTTP_GET_SERVICE_H__

#include <curl/curl.h>
#include <memory>
#include <mutex>
#include <string>
#include "switcher/exec-loop.hpp"
#include "switcher/information-tree.hpp"
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"

namespace switcher {
class HTTPGetService : public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(HTTPGetService);
  HTTPGetService(const std::string&);
  ~HTTPGetService();
  HTTPGetService(const HTTPGetService&) = delete;
  HTTPGetService& operator=(const HTTPGetService&) = delete;

 private:
  bool init() final;
  bool get_url(const std::string& url);
  bool make_properties(const InfoTree* tree);

  static int instance_count_;
  ExecLoop loop_{};
  std::string config_file_{};
  PContainer::prop_id_t config_file_id_;
  CURL* curl_{nullptr};
  std::map<std::string, bool> vals_{};
  // periodic task for repeating http get when property is on
  std::map<std::string, PeriodicTask::task_t> tasks_{};
  std::chrono::milliseconds ptask_period_ms_{1000};
  std::mutex ptask_mutex_{};
  std::unique_ptr<PeriodicTask> ptask_;
};

SWITCHER_DECLARE_PLUGIN(HTTPGetService);

}  // namespace switcher
#endif
