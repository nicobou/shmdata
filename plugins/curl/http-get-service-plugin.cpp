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

#include "./http-get-service-plugin.hpp"
#include "switcher/file-utils.hpp"
#include "switcher/information-tree-json.hpp"

namespace switcher {

int HTTPGetService::instance_count_ = 0;

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(HTTPGetService,
                                     "http-get-service",
                                     "HTTP Get service",
                                     "controller",
                                     "hid/device",
                                     "Use HTTP Get based service",
                                     "LGPL",
                                     "Nicolas Bouillot");

HTTPGetService::HTTPGetService(const std::string&)
    : config_file_id_(pmanage<MPtr(&PContainer::make_string)>(
          "config_file",
          [this](const std::string& val) {
            config_file_ = val;
            auto file_content = FileUtils::get_file_content(val);
            if (file_content.first.empty()) {
              g_message("ERROR: %s", file_content.second.c_str());
              return false;
            }
            auto tree = JSONSerializer::deserialize(file_content.first);
            if (!tree) {
              g_message("ERROR: %s is not a valid json file", val.c_str());
              return false;
            }
            if (!make_properties(tree.get())) return false;
            pmanage<MPtr(&PContainer::disable)>(config_file_id_, "service already loaded");
            return true;
          },
          [this]() { return config_file_; },
          "HTTPGet Description File",
          "This property allows for loading the command description",
          config_file_)),
      ptask_(std::make_unique<PeriodicTask>(
          [this]() {
            std::lock_guard<std::mutex> lock(ptask_mutex_);
            for (auto& it : tasks_) it.second();
          },
          ptask_period_ms_)) {}

bool HTTPGetService::init() {
  if (instance_count_ == 0) {
    if (0 != curl_global_init(CURL_GLOBAL_DEFAULT)) {
      g_warning("curl_global_init failled");
      return false;
    }
  }
  ++instance_count_;
  curl_ = curl_easy_init();
  return curl_;
}

HTTPGetService::~HTTPGetService() {
  curl_easy_cleanup(curl_);
  --instance_count_;
  if (instance_count_ == 0) curl_global_cleanup();
}

bool HTTPGetService::get_url(const std::string& url) {
  CURLcode res;
  curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
  /* Perform the request, res will get the return code */
  res = curl_easy_perform(curl_);
  /* Check for errors */
  if (res != CURLE_OK) {
    g_message("ERROR: curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    return false;
  }
  return true;
}

bool HTTPGetService::make_properties(const InfoTree* tree) {
  auto props = tree->get_child_keys(".");
  for (auto& it : props) {
    auto url = tree->branch_read_data<std::string>(it + ".url");
    if (url.empty()) {
      g_warning("could not get url from description");
      return false;
    }
    pmanage<MPtr(&PContainer::make_bool)>(it,
                                          [this, it, url](bool val) {
                                            std::lock_guard<std::mutex> lock(ptask_mutex_);
                                            vals_[it] = val;
                                            if (val) {
                                              tasks_[it] = [this, url]() { get_url(url); };
                                            } else {
                                              auto found = tasks_.find(it);
                                              if (found != tasks_.end()) tasks_.erase(found);
                                            }
                                            return true;
                                          },
                                          [this, it]() { return vals_[it]; },
                                          tree->branch_read_data<std::string>(it + ".name"),
                                          tree->branch_read_data<std::string>(it + ".descr"),
                                          true);
  }
  return true;
}

}  // namespace switcher
