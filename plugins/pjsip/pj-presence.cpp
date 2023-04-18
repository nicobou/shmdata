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

#include "./pj-presence.hpp"
#include <list>
#include <string>
#include "./pj-sip-plugin.hpp"
#include "switcher/infotree/json-serializer.hpp"
#include "switcher/utils/scope-exit.hpp"
#include "switcher/utils/string-utils.hpp"

namespace switcher {
namespace quiddities {

const std::map<PJPresence::SipStatus, std::string> PJPresence::SipStatusMap{
    {SipStatus::ONLINE, "online"},
    {SipStatus::OFFLINE, "offline"},
    {SipStatus::AWAY, "away"},
    {SipStatus::BUSY, "busy"},
    {SipStatus::UNKNOWN, "unknown"}
};

PJPresence::PJPresence() {
  // registering account
  using register_t = std::function<bool(std::string, std::string)>;
  SIPPlugin::this_->mmanage<MPtr(&method::MBag::make_method<register_t>)>(
      "register",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Register SIP Account",
                   "description" : "register a SIP account", 
                   "arguments" : [
                     {
                        "long name" : "SIP address",
                        "description" : "string"
                     }, {
                        "long name" : "SIP password",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
      [this](const std::string& login, const std::string& pass) {
        return register_account_wrapped(login, pass);
      });

  SIPPlugin::this_->mmanage<MPtr(&method::MBag::make_method<std::function<bool()>>)>(
      "unregister",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Unregister SIP Account",
                   "description" : "unregister SIP account",
                   "arguments" : []
                  }
              )"),
      [this]() { return unregister_account_wrapped(); });

  // buddies
  SIPPlugin::this_->mmanage<MPtr(&method::MBag::make_method<std::function<bool(std::string)>>)>(
      "add_buddy",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Add Buddy",
                   "description" : "add a buddy",
                   "arguments" : [
                     {
                        "long name" : "SIP User Name",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
      [this](const std::string& buddy) { return add_buddy_wrapped(buddy); });

  using set_name_t = std::function<bool(std::string, std::string)>;
  SIPPlugin::this_->mmanage<MPtr(&method::MBag::make_method<set_name_t>)>(
      "name_buddy",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Set Buddy Name", 
                   "description" :  "give a name to a buddy",
                   "arguments" : [
                     {
                        "long name" : "Name",
                        "description" : "string"
                     },
                     {
                        "long name" : "SIP URI",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
      [this](const std::string& name, const std::string& buddy) {
        return name_buddy_wrapped(name, buddy);
      });

  SIPPlugin::this_->mmanage<MPtr(&method::MBag::make_method<std::function<bool(std::string)>>)>(
      "del_buddy",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Del Buddy",
                   "description" :  "Delete a buddy",
                   "arguments" : [
                     {
                        "long name" : "SIP User Name",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
      [this](const std::string& name) { return del_buddy_wrapped(name); });

  // online status
  SIPPlugin::this_->pmanage<MPtr(&property::PBag::make_selection<>)>(
      "status",
      [this](const quiddity::property::IndexOrName& val) {
        status_.select(val);
        if (-1 == account_id_) {
          SIPPlugin::this_->sw_warning("cannot send online status when not registered");
          return true;
        }
        SIPPlugin::this_->pjsip_->run(
            [this]() { change_online_status(status_.get_current_index()); });
        return true;
      },
      [this]() { return status_.get(); },
      "Online Status",
      "Online Status",
      status_);
  SIPPlugin::this_->pmanage<MPtr(&property::PBag::make_string)>(
      "status-note",
      [this](const std::string& val) {
        custom_status_ = val;
        SIPPlugin::this_->pjsip_->run(
            [this]() { change_online_status(status_.get_current_index()); });
        return true;
      },
      [this]() { return custom_status_; },
      "Custom status note",
      "Custom status note",
      custom_status_);

  SIPPlugin::this_->pmanage<MPtr(&property::PBag::make_bool)>("sip-registration",
                                                              nullptr,
                                                              [this]() { return registered_; },
                                                              "Registered",
                                                              "Self SIP registration status",
                                                              registered_);
  SIPPlugin::this_->graft_tree(".self.", InfoTree::make(nullptr));

  SIPPlugin::this_->pmanage<MPtr(&property::PBag::make_bool)>(
      "lower-case-accounts",
      [this](const bool val) {
        lower_case_accounts_ = val;
        return true;
      },
      [this]() { return lower_case_accounts_; },
      "Lower Case Accounts",
      "Lower Case Accounts",
      lower_case_accounts_);
}

PJPresence::~PJPresence() {
  unregister_account(false);
  if (nullptr != acc_info_pool_) {
    pj_pool_release(acc_info_pool_);
    acc_info_pool_ = nullptr;
  }
}

bool PJPresence::register_account_wrapped(const std::string& user, const std::string& password) {
  if (user.empty() || password.empty()) {
    SIPPlugin::this_->sw_warning("register sip account missing user or domain or password");
    return false;
  }
  if (-1 == SIPPlugin::this_->transport_id_) {
    SIPPlugin::this_->sw_warning("cannot register, SIP port is not available ({})",
                                 std::to_string(SIPPlugin::this_->sip_port_));
    return false;
  }
  std::string tmp = user;
  if (lower_case_accounts_) stringutils::tolower(tmp);
  SIPPlugin::this_->pjsip_->run([&]() { register_account(tmp, std::string(password)); });
  SIPPlugin::this_->pmanage<MPtr(&property::PBag::notify)>(
      SIPPlugin::this_->pmanage<MPtr(&property::PBag::get_id)>("sip-registration"));
  if (registered_) return true;
  return false;
}

void PJPresence::register_account(const std::string& sip_user, const std::string& sip_password) {
  std::unique_lock<std::mutex> lock(registration_mutex_);
  // Register to SIP server by creating SIP account.
  //  pjsua_acc_config cfg;
  pj_status_t status;
  // extracting domain information from sip uri
  auto at = std::find(sip_user.begin(), sip_user.end(), '@');
  auto colon = std::find(sip_user.begin(), sip_user.end(), ':');
  if (sip_user.end() == at || sip_user.end() == (at + 1)) {
    SIPPlugin::this_->sw_warning("{}: invalid sip uri", std::string(__FUNCTION__));
    return;
  }
  if (nullptr != acc_info_pool_) {
    pj_pool_release(acc_info_pool_);
    acc_info_pool_ = nullptr;
  }
  acc_info_pool_ =
      pj_pool_create(&PJSIP::this_->cp_.factory, "account_config", 1024, 1024, nullptr);
  // setting pjsip account data structure
  pjsua_acc_config_default(&cfg_);
  cfg_.id = pj_strdup3(acc_info_pool_,
                       std::string("sip:" + std::string(sip_user.begin(), colon)  // + ";transport=tcp"
                                   )
                           .c_str());
  cfg_.reg_uri = pj_strdup3(acc_info_pool_,
                            std::string("sip:" + sip_user  // + ";transport=tcp"
                                        )
                                .c_str());
  cfg_.cred_count = 1;
  cfg_.cred_info[0].realm = pj_strdup3(acc_info_pool_, std::string(at + 1, colon).c_str());
  cfg_.cred_info[0].scheme = pj_strdup3(acc_info_pool_, "digest");
  cfg_.cred_info[0].username =
      pj_strdup3(acc_info_pool_, std::string(sip_user.begin(), at).c_str());
  cfg_.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
  cfg_.cred_info[0].data = pj_strdup3(acc_info_pool_, sip_password.c_str());
  cfg_.publish_enabled = PJ_TRUE;
  // pjsua_acc_set_registration (account_id_, PJ_TRUE) or:
  cfg_.register_on_acc_add = PJ_TRUE;
  status = pjsua_acc_add(&cfg_, PJ_TRUE, &account_id_);
  if (status != PJ_SUCCESS || !pjsua_acc_is_valid(account_id_)) {
    account_id_ = -1;
    return;
  }
  pjsua_acc_set_user_data(account_id_, this);
  registration_cond_.wait_for(lock, std::chrono::seconds(5));
  if (registered_) {
    change_online_status(PJPresence::AVAILABLE);
    // notifying sip registration status
    SIPPlugin::this_->graft_tree(".self.", InfoTree::make(std::string(sip_user.begin(), colon)));
    sip_local_user_ = std::string("sip:") + std::string(sip_user.begin(), colon) + ":" +
                      std::to_string(SIPPlugin::this_->sip_port_);
  }
}

bool PJPresence::unregister_account_wrapped() {
  SIPPlugin::this_->pjsip_->run([&]() { unregister_account(); });
  if (-1 != account_id_ || !pjsua_acc_is_valid(account_id_)) return false;
  return true;
}

void PJPresence::unregister_account(bool notify_tree) {
  std::unique_lock<std::mutex> lock(registration_mutex_);

  if (-1 == account_id_ || !pjsua_acc_is_valid(account_id_)) {
    SIPPlugin::this_->sw_warning("no account to unregister");
    return;
  }
  change_online_status(PJPresence::OFFLINE);
  if (PJ_SUCCESS != pjsua_acc_del(account_id_)) {
    SIPPlugin::this_->sw_warning("error when unregistering account");
    return;
  }
  registration_cond_.wait_for(lock, std::chrono::seconds(3));

  pj_status_t status = PJ_SUCCESS;
  while (buddy_id_.begin() != buddy_id_.end()) {
    auto it = buddy_id_.begin();
    status = pjsua_buddy_del(it->second);
    if (status != PJ_SUCCESS) {
      SIPPlugin::this_->sw_warning("cannot remove buddy");
      return;
    }
    if (notify_tree) SIPPlugin::this_->prune_tree(".buddies." + std::to_string(it->second));
    buddy_id_.erase(it);
  }
  account_id_ = -1;
  sip_local_user_.clear();
  registered_ = false;
  if (notify_tree) {
    SIPPlugin::this_->graft_tree(".self.", InfoTree::make(nullptr));
    SIPPlugin::this_->pmanage<MPtr(&property::PBag::notify)>(
        SIPPlugin::this_->pmanage<MPtr(&property::PBag::get_id)>("sip-registration"));
  }

  return;
}

void PJPresence::add_buddy(const std::string& user) {
  pjsua_buddy_config buddy_cfg;
  pjsua_buddy_id buddy_id;
  pj_status_t status = PJ_SUCCESS;

  if (-1 == account_id_) {
    SIPPlugin::this_->sw_warning("cannot add buddy with invalid account");
    return;
  }
  std::string sip_user = user;
  if (lower_case_accounts_) stringutils::tolower(sip_user);
  std::string buddy_full_uri("sip:" + sip_user  // + ";transport=tcp"
                             );
  if (pjsua_verify_url(buddy_full_uri.c_str()) != PJ_SUCCESS) {
    SIPPlugin::this_->sw_warning("invalid buddy URI (sip:{})", sip_user);
    return;
  }

  sip_user = std::string(sip_user.begin(), std::find(sip_user.begin(), sip_user.end(), ':'));
  if (buddy_id_.end() != buddy_id_.find(sip_user)) {
    SIPPlugin::this_->sw_warning("buddy {} already added", sip_user);
    return;
  }

  pj_bzero(&buddy_cfg, sizeof(pjsua_buddy_config));
  pj_cstr(&buddy_cfg.uri, buddy_full_uri.c_str());
  buddy_cfg.subscribe = PJ_TRUE;
  buddy_cfg.user_data = this;
  status = pjsua_buddy_add(&buddy_cfg, &buddy_id);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->sw_warning("buddy not found");
    return;
  }

  SIPPlugin::this_->sw_debug("buddy added");

  buddy_id_[sip_user] = buddy_id;

  // get the quiddity infotree and declare paths
  auto tree = SIPPlugin::this_->get_tree(".");
  const std::string buddiespath = ".buddies";
  const std::string buddypath = ".buddies." + std::to_string(buddy_id);

  // check for `buddies` branch
  if (!tree->branch_is_array(buddiespath)) {
    tree->graft(buddiespath, InfoTree::make());
    tree->tag_as_array(buddiespath, true);
  }

  // graft buddy data
  tree->graft(buddypath + ".uri", InfoTree::make(sip_user));
  tree->graft(buddypath + ".send_status",
              InfoTree::make(PJCall::SendRecvStatusMap.at(PJCall::SendRecvStatus::DISCONNECTED)));
  tree->graft(buddypath + ".recv_status",
              InfoTree::make(PJCall::SendRecvStatusMap.at(PJCall::SendRecvStatus::DISCONNECTED)));

  // graft infotree
  SIPPlugin::this_->graft_tree(".", tree);

  // add our new buddy to the infotree whitelist and then trigger a callback to confirm that our
  // buddy has been authorized
  SIPPlugin::this_->white_list_->add(sip_user, [buddypath, tree](bool authorized) {
    SIPPlugin::this_->graft_tree(buddypath + ".whitelisted", InfoTree::make(authorized));
  });

  return;
}

void PJPresence::del_buddy(const std::string& user) {
  pj_status_t status = PJ_SUCCESS;
  if (pjsua_verify_url(std::string("sip:" + user).c_str()) != PJ_SUCCESS) {
    SIPPlugin::this_->sw_warning("invalid buddy URI ({}) for deletion", user);
    return;
  }
  std::string sip_user = user;
  if (lower_case_accounts_) stringutils::tolower(sip_user);
  auto it = buddy_id_.find(sip_user);
  if (buddy_id_.end() == it) {
    SIPPlugin::this_->sw_warning("{} is not in buddy list, cannot delete", sip_user);
    return;
  }
  status = pjsua_buddy_del(it->second);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->sw_warning("cannot remove buddy");
    return;
  }
  SIPPlugin::this_->white_list_->remove(sip_user);
  SIPPlugin::this_->prune_tree(".buddies." + std::to_string(it->second));
  buddy_id_.erase(it);
  SIPPlugin::this_->sw_debug("buddy removed");
  return;
}

bool PJPresence::add_buddy_wrapped(const std::string& buddy_uri) {
  if (-1 == SIPPlugin::this_->transport_id_) {
    SIPPlugin::this_->sw_warning("cannot add buddy without connection to server");
    return false;
  }

  SIPPlugin::this_->pjsip_->run([&]() { add_buddy(buddy_uri); });
  return true;
}

bool PJPresence::del_buddy_wrapped(const std::string& buddy_uri) {
  SIPPlugin::this_->pjsip_->run([&]() { del_buddy(buddy_uri); });
  return true;
}

void PJPresence::name_buddy(std::string name, std::string sip_user) {
  if (pjsua_verify_url(std::string("sip:" + sip_user).c_str()) != PJ_SUCCESS) {
    SIPPlugin::this_->sw_warning("invalid buddy URI ({}) when giving a nick name", sip_user);
    return;
  }

  auto it = buddy_id_.find(sip_user);
  if (buddy_id_.end() == it) {
    SIPPlugin::this_->sw_warning("{} is not in buddy list", sip_user);
    return;
  }
  SIPPlugin::this_->graft_tree(".buddies." + std::to_string(it->second) + ".name",
                               InfoTree::make(std::string(name)));
  return;
}

bool PJPresence::name_buddy_wrapped(const std::string& name, const std::string& buddy_uri) {
  std::string bud(buddy_uri);
  if (lower_case_accounts_) stringutils::tolower(bud);
  SIPPlugin::this_->pjsip_->run([&]() { name_buddy(std::string(name), bud); });
  return true;
}

void PJPresence::on_registration_state(pjsua_acc_id acc_id, pjsua_reg_info* info) {
  PJPresence* context = static_cast<PJPresence*>(pjsua_acc_get_user_data(acc_id));
  if (nullptr == context || !pjsua_acc_is_valid(acc_id)) {
    SIPPlugin::this_->sw_warning("SIP registration failed");
    SIPPlugin::this_->pmanage<MPtr(&property::PBag::notify)>(
        SIPPlugin::this_->pmanage<MPtr(&property::PBag::get_id)>("sip-registration"));
    return;
  }
  std::unique_lock<std::mutex> lock(context->registration_mutex_);
  // SIP code higher to 299 are error code
  if (PJ_SUCCESS != info->cbparam->status || info->cbparam->code > 299) {
    if (info->cbparam->code == 408) {
      SIPPlugin::this_->sw_warning(
          "registration failed (timeout), SIP server did not answer ({})",
          std::string(info->cbparam->reason.ptr, static_cast<int>(info->cbparam->reason.slen)));
    } else {
      SIPPlugin::this_->sw_warning(
          "registration failed ({})",
          std::string(info->cbparam->reason.ptr, static_cast<int>(info->cbparam->reason.slen)));
    }
    if (-1 != context->account_id_) {
      pj_status_t status = pjsua_acc_del(context->account_id_);
      if (PJ_SUCCESS != status)
        SIPPlugin::this_->sw_warning("error when deleting account after registration failed");
      context->account_id_ = -1;
    }
    context->registered_ = false;
  } else {
    if (199 < info->cbparam->code && 300 > info->cbparam->code)
      context->registered_ = true;
    else
      context->registered_ = false;
  }
  SIPPlugin::this_->sw_debug("registration SIP status code {}",
                             std::to_string(info->cbparam->code));
  context->registration_cond_.notify_one();
}

void PJPresence::on_buddy_state(pjsua_buddy_id buddy_id) {
  PJPresence* context = static_cast<PJPresence*>(pjsua_buddy_get_user_data(buddy_id));
  if (nullptr == context) return;
  pjsua_buddy_info info;
  pjsua_buddy_get_info(buddy_id, &info);
  std::string status(SipStatusMap.at(SipStatus::UNKNOWN));
  switch (info.status) {
    case PJSUA_BUDDY_STATUS_UNKNOWN:
      break;
    case PJSUA_BUDDY_STATUS_ONLINE:
      status = SipStatusMap.at(SipStatus::ONLINE);
      break;
    case PJSUA_BUDDY_STATUS_OFFLINE:
      status = SipStatusMap.at(SipStatus::OFFLINE);
      break;
    default:
      break;
  }
  if (PJRPID_ACTIVITY_AWAY == info.rpid.activity) status = SipStatusMap.at(SipStatus::AWAY);
  if (PJRPID_ACTIVITY_BUSY == info.rpid.activity) status = SipStatusMap.at(SipStatus::BUSY);

  InfoTree::ptr tree =
      SIPPlugin::this_->get_tree(std::string(".buddies." + std::to_string(buddy_id)));
  if (!tree) tree = InfoTree::make();
  // writing status and state
  if (std::string(info.status_text.ptr, (size_t)info.status_text.slen) == "Offline")
    tree->graft(".status", InfoTree::make(SipStatusMap.at(SipStatus::OFFLINE)));
  else
    tree->graft(".status", InfoTree::make(status));
  tree->graft(".status_text",
              InfoTree::make(std::string(info.status_text.ptr, (size_t)info.status_text.slen)));
  tree->graft(".subscription_state", InfoTree::make(std::string(info.sub_state_name)));
  // replacing old one
  SIPPlugin::this_->graft_tree(std::string(".buddies." + std::to_string(buddy_id)), tree);
}

void PJPresence::change_online_status(int status) {
  if (custom_status_ == "Offline") custom_status_.clear();
  if (-1 == account_id_) return;
  pjrpid_element elem;
  pj_bzero(&elem, sizeof(elem));
  elem.type = PJRPID_ELEMENT_TYPE_PERSON;
  bool has_custom_status = true;
  char* tmp = nullptr;
  On_scope_exit {
    if (nullptr != tmp) g_free(tmp);
  };
  if (custom_status_.empty() || custom_status_ == "" || custom_status_ == "Available" ||
      custom_status_ == "Away" || custom_status_ == "Busy") {
    has_custom_status = false;
  } else {
    tmp = g_strdup(custom_status_.c_str());
    elem.note = pj_str(tmp);
  }
  std::string tmp_status;
  switch (status) {
    case AVAILABLE:
      tmp_status = "Available";
      break;
    case BUSY:
      elem.activity = PJRPID_ACTIVITY_BUSY;
      tmp_status = "Busy";
      break;
    case AWAY:
      elem.activity = PJRPID_ACTIVITY_AWAY;
      tmp_status = "Away";
      break;
    case OFFLINE:
      // use away since we do not want to actually go offline
      elem.activity = PJRPID_ACTIVITY_AWAY;
      tmp_status = "Offline";
      break;
  }
  if (!has_custom_status && !tmp_status.empty()) {
    pj_cstr(&elem.note, tmp_status.c_str());
    custom_status_ = tmp_status;
  }
  pjsua_acc_set_online_status2(account_id_, PJ_TRUE, &elem);
}

/*
 * Handler registration status has changed.
 */
void PJPresence::on_reg_state(pjsua_acc_id acc_id) {
  PJ_UNUSED_ARG(acc_id);
  // Log already written.
}

/*
 * Handler for incoming presence subscription request
 */
void PJPresence::on_incoming_subscribe(pjsua_acc_id acc_id,
                                       pjsua_srv_pres* srv_pres,
                                       pjsua_buddy_id buddy_id,
                                       const pj_str_t* from,
                                       pjsip_rx_data* rdata,
                                       pjsip_status_code* code,
                                       pj_str_t* reason,
                                       pjsua_msg_data* msg_data) {
  /* Just accept the request (the default behavior) */
  PJ_UNUSED_ARG(acc_id);
  PJ_UNUSED_ARG(srv_pres);
  PJ_UNUSED_ARG(buddy_id);
  PJ_UNUSED_ARG(from);
  PJ_UNUSED_ARG(rdata);
  PJ_UNUSED_ARG(code);
  PJ_UNUSED_ARG(reason);
  PJ_UNUSED_ARG(msg_data);
}

/*
 * Subscription state has changed.
 */
void PJPresence::on_buddy_evsub_state(pjsua_buddy_id /*buddy_id*/,
                                      pjsip_evsub* sub,
                                      pjsip_event* event) {
  char event_info[80];
  PJ_UNUSED_ARG(sub);
  event_info[0] = '\0';
  if (event->type == PJSIP_EVENT_TSX_STATE && event->body.tsx_state.type == PJSIP_EVENT_RX_MSG) {
    pjsip_rx_data* rdata = event->body.tsx_state.src.rdata;
    snprintf(event_info, sizeof(event_info), " (RX %s)", pjsip_rx_data_get_info(rdata));
  }
}

pjsua_buddy_id PJPresence::get_id_from_buddy_name(const std::string& name) {
  auto bud =
      std::find_if(buddy_id_.begin(), buddy_id_.end(), [&name](decltype(*buddy_id_.begin())& it) {
        return stringutils::starts_with(it.first, name + "@");
      });
  if (buddy_id_.end() != bud) return bud->second;
  return PJSUA_INVALID_ID;
}

}  // namespace quiddities
}  // namespace switcher
