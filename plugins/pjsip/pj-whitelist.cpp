/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "./pj-whitelist.hpp"
#include "./pj-sip-plugin.hpp"
#include "switcher/information-tree-json.hpp"

namespace switcher {

PJWhiteList::PJWhiteList()
    : mode_id_(SIPPlugin::this_->pmanage<MPtr(&PContainer::make_selection<>)>(
          "mode",
          [this](const IndexOrName& val) {
            mode_.select(val);
            return true;
          },
          [this]() { return mode_.get(); },
          "Access list mode",
          "Control which incoming calls are accepted",
          mode_)),
      everybody_(mode_.get_index("everybody")) {
  using authorize_t = std::function<bool(std::string, bool)>;
  SIPPlugin::this_->mmanage<MPtr(&MContainer::make_method<authorize_t>)>(
      "authorize",
      JSONSerializer::deserialize(
          R"(
                  {
                   "name" : "Authorized Incoming Call From",
                   "description" : "authorize incoming call from an buddy",
                   "arguments" : [
                     {
                        "long name" : "SIP url",
                        "description" : "string"
                     },
                     {
                        "long name" : "Authorized",
                        "description" : "true or false"
                     }
                   ]
                  }
              )"),
      [&](const std::string& url, bool auth) { return authorize_buddy_cb(url, auth); });
}

bool PJWhiteList::authorize_buddy_cb(const std::string& sip_url, bool authorized) {
  if (authorize_buddy(sip_url, authorized)) return true;
  return false;
}

bool PJWhiteList::authorize_buddy(const std::string& sip_url, bool authorize) {
  auto contact = std::string(sip_url.begin(), std::find(sip_url.begin(), sip_url.end(), ':'));
  auto found = authorizations_.find(contact);
  if (authorizations_.end() == found) {
    return false;
  }
  found->second = authorize;
  on_auth_cbs_[contact](authorize);
  return true;
}

bool PJWhiteList::add(const std::string& sip_url, on_authorization_updated_t cb) {
  if (!cb) return false;
  if (authorizations_.end() != authorizations_.find(sip_url)) {
    return false;
  }
  authorizations_[sip_url] = default_authorization_;
  on_auth_cbs_[sip_url] = cb;
  cb(default_authorization_);
  return true;
}

bool PJWhiteList::remove(const std::string& sip_url) {
  {
    auto found = authorizations_.find(sip_url);
    if (authorizations_.end() == found) return false;
    authorizations_.erase(found);
  }
  {
    auto found = on_auth_cbs_.find(sip_url);
    if (on_auth_cbs_.end() != found) on_auth_cbs_.erase(found);
  }
  return true;
}

bool PJWhiteList::is_authorized(const std::string& sip_url) const {
  if (mode_.get_current_index() == everybody_) return true;
  const auto& found = authorizations_.find(sip_url);
  if (authorizations_.cend() == found) return false;
  return found->second;
}

}  // namespace switcher
