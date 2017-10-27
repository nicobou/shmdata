/*
 * This file is part of switcher-protocol-mapper.
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
#include "protocol-curl.hpp"

namespace switcher {

std::atomic<int> ProtocolCurl::instance_count_{0};

ProtocolCurl::ProtocolCurl(Quiddity* quid, const InfoTree* tree) : ProtocolReader(quid, tree) {
  if (instance_count_ == 0) {
    if (0 != curl_global_init(CURL_GLOBAL_DEFAULT)) {
      quid->warning("curl_global_init failed");
      return;
    }
  }
  ++instance_count_;
  curl_ = curl_easy_init();
}

ProtocolCurl::~ProtocolCurl() {
  curl_easy_cleanup(curl_);
  --instance_count_;
  if (instance_count_ == 0) curl_global_cleanup();
}

bool ProtocolCurl::curl_request(const std::string& url) {
  CURLcode res;
  curl_easy_setopt(curl_, CURLOPT_TIMEOUT_MS, ProtocolCurl::kTimeout);
  curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
  /* Perform the request, res will get the return code */
  res = curl_easy_perform(curl_);
  /* Check for errors */
  if (res != CURLE_OK) {
#ifdef DEBUG
    std::cerr << " curl_easy_perform() failed: " << curl_easy_strerror(res) << '\n';
#endif
    return false;
  }
  return true;
}

bool ProtocolCurl::make_properties(Quiddity* quid, const InfoTree* tree) {
  auto props = tree->get_child_keys(".");
  for (auto& it : props) {
    auto url = tree->branch_read_data<std::string>(it + ".url");

    if (url.empty()) {
      quid->warning("Could not get url from description (protocol-mapper curl)");
      return false;
    }

    auto continuous = false;
    if (continuous_) {
      continuous = tree->branch_read_data<std::string>(it + ".continuous") == "true";
    }

    quid->pmanage<MPtr(&PContainer::make_bool)>(
        it,
        [this, it, url, continuous](bool val) {
          if (continuous) vals_[it] = val;
          if (val) {
            if (continuous) {
              std::lock_guard<std::mutex> lock(ptask_mutex_);
              tasks_.insert(std::make_pair(
                  it, ProtocolReader::Command([this, url]() { curl_request(url); }, continuous)));
            } else {
              curl_request(url);
            }
          } else {
            if (continuous) {
              std::lock_guard<std::mutex> lock(ptask_mutex_);
              auto found = tasks_.find(it);
              if (found != tasks_.end()) tasks_.erase(found);
            }
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
}
