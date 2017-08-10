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
  SIPPlugin::this_->install_method(
      "Authorized Incoming Call From",            // long name
      "authorize",                                // name
      "authorize incoming call from an buddy",    // description
      "success",                                  // return desc
      Method::make_arg_description("SIP url",     // long name
                                   "url",         // name
                                   "string",      // description
                                   "Authorized",  // long name
                                   "authorized",  // name
                                   "boolean",     // description
                                   nullptr),
      (Method::method_ptr)&authorize_buddy_cb,
      G_TYPE_BOOLEAN,
      Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_BOOLEAN, nullptr),
      this);
}

gboolean PJWhiteList::authorize_buddy_cb(gchar* sip_url, gboolean authorized, void* user_data) {
  PJWhiteList* context = static_cast<PJWhiteList*>(user_data);
  if (context->authorize_buddy(sip_url, authorized)) return TRUE;
  return FALSE;
}

bool PJWhiteList::authorize_buddy(const std::string& sip_url, bool authorize) {
  auto found = authorizations_.find(sip_url);
  if (authorizations_.end() == found) {
    return false;
  }
  found->second = authorize;
  on_auth_cbs_[sip_url](authorize);
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
