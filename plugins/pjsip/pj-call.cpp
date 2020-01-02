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

#include "./pj-call.hpp"
#include "./pj-call-utils.hpp"
#include "./pj-sip-plugin.hpp"
#include "switcher/gst/sdp-utils.hpp"
#include "switcher/infotree/information-tree-json.hpp"
#include "switcher/utils/net-utils.hpp"
#include "switcher/utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {

char* pjcall_pjsip_module_name = strdup("mod-siprtpapp");

pjsip_module PJCall::mod_siprtp_ = {
    nullptr,
    nullptr,                          /* prev, next. */
    pj_str(pjcall_pjsip_module_name), /* Name.       */
    -1,                               /* Id          */
    // (before PJSIP_MOD_PRIORITY_UA_PROXY_LAYER):
    30,             /* Priority         */
    nullptr,        /* load()           */
    nullptr,        /* start()          */
    nullptr,        /* stop()           */
    nullptr,        /* unload()         */
    &on_rx_request, /* on_rx_request()  */
    nullptr,        /* on_rx_response() */
    nullptr,        /* on_tx_request.   */
    nullptr,        /* on_tx_response() */
    nullptr,        /* on_tsx_state()   */
};

PJCall::PJCall() {
  pj_status_t status;
  local_ips_ = NetUtils::get_ips();
  for (auto& it : local_ips_)
    SIPPlugin::this_->debug("Local IP found for interface %: %", it.first, it.second);
  for (auto& it : local_ips_) {
    if (0 != std::string(it.second, 0, 4).compare("127.")) pj_cstr(&local_addr_, it.second.c_str());
  }
  if (nullptr == local_addr_.ptr) pj_cstr(&local_addr_, "127.0.0.1");
  /*  Init invite session module. */
  {
    pjsip_inv_callback inv_cb;
    /* Init the callback for INVITE session: */
    pj_bzero(&inv_cb, sizeof(inv_cb));
    inv_cb.on_rx_offer = &call_on_rx_offer;
    inv_cb.on_state_changed = &call_on_state_changed;
    inv_cb.on_new_session = &call_on_forked;
    inv_cb.on_media_update = &call_on_media_update;
    // unregister/shutdown default invite module
    status = pjsip_endpt_unregister_module(PJSIP::this_->sip_endpt_, pjsip_inv_usage_instance());
    if (status != PJ_SUCCESS)
      SIPPlugin::this_->warning("unregistering default invite module failed");
    /* Initialize invite session module:  */
    status = pjsip_inv_usage_init(PJSIP::this_->sip_endpt_, &inv_cb);
    if (status != PJ_SUCCESS) SIPPlugin::this_->warning("Init invite session module failed");
  }
  pjsip_100rel_init_module(PJSIP::this_->sip_endpt_);
  /* Register our module to receive incoming requests. */
  status = pjsip_endpt_register_module(PJSIP::this_->sip_endpt_, &mod_siprtp_);
  if (status != PJ_SUCCESS) SIPPlugin::this_->warning("Register mod_siprtp_ failed");
  // registering codecs
  status = PJCodec::install_codecs();
  if (status != PJ_SUCCESS) SIPPlugin::this_->warning("Install codecs failed");
  // properties and methods for user
  SIPPlugin::this_->mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
      "send",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Send to a contact",
                   "description" : "invite a contact to receive data",
                   "arguments" : [
                     {
                        "long name" : "SIP url", 
                        "description" : "url"
                     }
                   ]
                  }
              )"),
      [this](const std::string& url) { return send_to(url); });

  SIPPlugin::this_->mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
      "hang-up",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Hang Up",
                   "description" : "Hang up a call",
                   "arguments" : [
                     {
                        "long name" : "SIP url", 
                        "description" : "url"
                     }
                   ]
                  }
              )"),
      [this](const std::string& url) { return hang_up(url); });

  using attach_t = std::function<bool(std::string, std::string, bool)>;
  SIPPlugin::this_->mmanage<MPtr(&MContainer::make_method<attach_t>)>(
      "attach_shmdata_to_contact",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Attach Shmdata To Contact",
                   "description" : "Register a shmdata for this contact",
                   "arguments" : [
                     {
                        "long name" : "Shmdata Path", 
                        "description" : "string"
                     }, {
                        "long name" : "Contact URI",
                        "description" : "string"
                     }, {
                        "long name" : "Contact URI",
                        "description" : "true or false"
                     }
                   ]
                  }
              )"),
      [this](const std::string& shmpath, const std::string& contact, bool attach) {
        return attach_shmdata_to_contact(shmpath, contact, attach);
      });
}

void PJCall::finalize_calls() {
  std::vector<std::string> uris;

  std::lock_guard<std::mutex> out_lock(SIPPlugin::this_->sip_calls_->finalize_outgoing_calls_m_);
  std::lock_guard<std::mutex> inc_lock(SIPPlugin::this_->sip_calls_->finalize_incoming_calls_m_);

  // Hangup outgoing calls
  for (auto& it : outgoing_call_) uris.push_back(it->peer_uri);
  for (auto& it : uris) hang_up(it);

  uris.clear();

  // Hangup incoming calls
  for (auto& it : incoming_call_) uris.push_back(it->peer_uri);
  for (auto& it : uris) hang_up(it);

  can_create_calls_ = false;
}

/* Callback to be called to handle incoming requests outside dialogs: */
pj_bool_t PJCall::on_rx_request(pjsip_rx_data* rdata) {
  /* Ignore strandled ACKs (must not send respone) */
  if (rdata->msg_info.msg->line.req.method.id == PJSIP_ACK_METHOD) return PJ_FALSE;
  /* Respond (statelessly) any non-INVITE requests with 500  */
  if (rdata->msg_info.msg->line.req.method.id != PJSIP_INVITE_METHOD) {
    return PJ_FALSE;
    // pj_str_t reason;
    // pj_cstr(&reason, "Unsupported Operation");
    // pjsip_endpt_respond_stateless(rdata->tp_info.transport->endpt,
    //                               rdata, 500, &reason, nullptr, nullptr);
    // return PJ_TRUE;
  }
  /* Handle incoming INVITE */
  process_incoming_call(rdata);
  /* Done */
  return PJ_TRUE;
}

void PJCall::on_inv_state_disconnected(call_t* call, pjsip_inv_session* inv, pjsua_buddy_id id) {
  SIPPlugin::this_->debug("Call disconnected. Reason=% (%)",
                          std::to_string(inv->cause),
                          std::string(inv->cause_text.ptr, static_cast<int>(inv->cause_text.slen)));
  if (!release_outgoing_call(call, id)) release_incoming_call(call, id);
}

bool PJCall::release_incoming_call(call_t* call, pjsua_buddy_id id) {
  std::lock_guard<std::mutex> lock(SIPPlugin::this_->sip_calls_->call_m_);

  auto& calls = SIPPlugin::this_->sip_calls_->incoming_call_;
  auto it = std::find_if(calls.begin(), calls.end(), [&call](const std::unique_ptr<call_t>& c) {
    return c->inv == call->inv;
  });
  if (calls.end() == it) return false;
  // updating recv status in the tree
  InfoTree::ptr tree =
      SIPPlugin::this_->prune_tree(std::string(".buddies." + std::to_string(id)),
                                   false);  // do not signal since the branch will be re-grafted
  if (!tree) {
    SIPPlugin::this_->warning("cannot find buddy information tree, call status update cancelled");
  } else {
    tree->graft(std::string(".recv_status."), InfoTree::make("disconnected"));
    SIPPlugin::this_->graft_tree(std::string(".buddies." + std::to_string(id)), tree);
  }

  if (SIPPlugin::this_->sip_calls_->is_hanging_up_) {
    SIPPlugin::this_->sip_calls_->call_action_done_ = true;
    SIPPlugin::this_->sip_calls_->call_cv_.notify_all();
  }
  // cleaning possible related extshmsrcs
  SIPPlugin::this_->remove_exposed_quiddities((*it)->peer_uri);

  // removing call
  (*it)->inv->mod_data[mod_siprtp_.id] = nullptr;
  calls.erase(it);

  return true;
}

bool PJCall::release_outgoing_call(call_t* call, pjsua_buddy_id id) {
  std::lock_guard<std::mutex> lock(SIPPlugin::this_->sip_calls_->call_m_);
  auto& calls = SIPPlugin::this_->sip_calls_->outgoing_call_;
  auto it = std::find_if(calls.begin(), calls.end(), [&call](const std::unique_ptr<call_t>& c) {
    return c->inv == call->inv;
  });
  if (calls.end() == it) return false;
  // removing destination to siprtp
  for (auto& media : (*it)->media) {
    if (0 != media.cb_id) {
      auto reader = SIPPlugin::this_->sip_calls_->readers_.find(media.shm_path_to_send);
      if (reader != SIPPlugin::this_->sip_calls_->readers_.cend() && reader->second) {
        reader->second->remove_cb(media.cb_id);
      }
    }
  }

  // updating call status in the tree
  InfoTree::ptr tree =
      SIPPlugin::this_->prune_tree(std::string(".buddies." + std::to_string(id)),
                                   false);  // do not signal since the branch will be re-grafted
  if (!tree) {
    SIPPlugin::this_->warning("cannot find buddy information tree, call status update cancelled");
  } else {
    tree->graft(std::string(".send_status."), InfoTree::make("disconnected"));
    SIPPlugin::this_->graft_tree(std::string(".buddies." + std::to_string(id)), tree);
  }
  // removing call
  (*it)->inv->mod_data[mod_siprtp_.id] = nullptr;
  calls.erase(it);
  if (SIPPlugin::this_->sip_calls_->is_hanging_up_ || SIPPlugin::this_->sip_calls_->is_calling_) {
    SIPPlugin::this_->sip_calls_->call_action_done_ = true;
    SIPPlugin::this_->sip_calls_->call_cv_.notify_all();
  }
  return true;
}

void PJCall::on_inv_state_confirmed(call_t* call, pjsip_inv_session* /*inv*/, pjsua_buddy_id id) {
  SIPPlugin::this_->debug("Call connected");
  // updating call status in the tree
  InfoTree::ptr tree =
      SIPPlugin::this_->prune_tree(std::string(".buddies." + std::to_string(id)),
                                   false);  // do not signal since the branch will be re-grafted
  if (!tree) {
    SIPPlugin::this_->warning("cannot find buddy information tree, call status update cancelled");
    return;
  }
  auto& calls = SIPPlugin::this_->sip_calls_->outgoing_call_;
  auto it = std::find_if(calls.begin(), calls.end(), [&call](const std::unique_ptr<call_t>& c) {
    return c->inv == call->inv;
  });
  if (SIPPlugin::this_->sip_calls_->is_calling_) {
    std::unique_lock<std::mutex> lock(SIPPlugin::this_->sip_calls_->call_m_);
    SIPPlugin::this_->sip_calls_->call_action_done_ = true;
    SIPPlugin::this_->sip_calls_->call_cv_.notify_all();
  }
  if (calls.end() != it)
    tree->graft(std::string(".send_status."), InfoTree::make("calling"));
  else
    tree->graft(std::string(".recv_status."), InfoTree::make("receiving"));
  SIPPlugin::this_->graft_tree(std::string(".buddies." + std::to_string(id)), tree);
}

void PJCall::on_inv_state_early(call_t* call, pjsip_inv_session* inv, pjsua_buddy_id id) {
  PJCall::on_inv_state_connecting(call, inv, id);
}

void PJCall::on_inv_state_connecting(call_t* call, pjsip_inv_session* /*inv*/, pjsua_buddy_id id) {
  // updating call status in the tree
  InfoTree::ptr tree =
      SIPPlugin::this_->prune_tree(std::string(".buddies." + std::to_string(id)),
                                   false);  // do not signal since the branch will be re-grafted
  if (!tree) {
    SIPPlugin::this_->warning("cannot find buddy information tree, call status update cancelled");
    return;
  }
  auto& calls = SIPPlugin::this_->sip_calls_->outgoing_call_;
  auto it = std::find_if(calls.begin(), calls.end(), [&call](const std::unique_ptr<call_t>& c) {
    return c->inv == call->inv;
  });
  if (calls.end() != it)
    tree->graft(std::string(".send_status."), InfoTree::make("connecting"));
  else
    tree->graft(std::string(".recv_status."), InfoTree::make("connecting"));
  SIPPlugin::this_->graft_tree(std::string(".buddies." + std::to_string(id)), tree);
}

/* Callback to be called when invite session's state has changed: */
void PJCall::call_on_state_changed(pjsip_inv_session* inv, pjsip_event* /*e*/) {
  if (!SIPPlugin::this_ || !SIPPlugin::this_->sip_calls_.get()) {
    return;
  }

  call_t* call = (call_t*)inv->mod_data[mod_siprtp_.id];
  switch (inv->state) {
    case PJSIP_INV_STATE_DISCONNECTED:
      break;
    case PJSIP_INV_STATE_CONFIRMED:
      break;
    case PJSIP_INV_STATE_EARLY:
      break;
    case PJSIP_INV_STATE_CONNECTING:
      break;
    case PJSIP_INV_STATE_NULL:
      break;
    case PJSIP_INV_STATE_CALLING:
      break;
    case PJSIP_INV_STATE_INCOMING:
      break;
    default:
      break;
  }

  if (!call) {
    SIPPlugin::this_->warning("%, null call in invite", std::string(__FUNCTION__));
    return;
  }
  // finding id of the buddy related to the call
  auto endpos = call->peer_uri.find('@');
  auto beginpos = call->peer_uri.find("sip:");
  if (0 == beginpos)
    beginpos = 4;
  else
    beginpos = 0;
  auto id = SIPPlugin::this_->sip_presence_->get_id_from_buddy_name(
      std::string(call->peer_uri, beginpos, endpos));
  if (PJSUA_INVALID_ID == id) {
    SIPPlugin::this_->warning("buddy not found: cannot update call status (%)", call->peer_uri);
    return;
  }
  switch (inv->state) {
    case PJSIP_INV_STATE_DISCONNECTED:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_DISCONNECTED");
      PJCall::on_inv_state_disconnected(call, inv, id);
      break;
    case PJSIP_INV_STATE_CONFIRMED:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_CONFIRMED");
      PJCall::on_inv_state_confirmed(call, inv, id);
      break;
    case PJSIP_INV_STATE_EARLY:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_EARLY");
      PJCall::on_inv_state_early(call, inv, id);
      break;
    case PJSIP_INV_STATE_CONNECTING:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_CONNECTING");
      PJCall::on_inv_state_connecting(call, inv, id);
      break;
    case PJSIP_INV_STATE_NULL:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_NULL");
      break;
    case PJSIP_INV_STATE_CALLING:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_CALLING");
      break;
    case PJSIP_INV_STATE_INCOMING:
      SIPPlugin::this_->debug("PJSIP_INV_STATE_INCOMING");
      break;
    default:
      SIPPlugin::this_->debug("%, unhandled invite state", std::string(__FUNCTION__));
      break;
  }
}

/* Callback to be called when dialog has forked: */
void PJCall::call_on_forked(pjsip_inv_session* /*inv*/, pjsip_event* /*e*/) {}

/* Callback to be called when SDP negotiation is done in the call: */
void PJCall::call_on_media_update(pjsip_inv_session* inv, pj_status_t status) {
  const pjmedia_sdp_session *local_sdp, *remote_sdp;
  call_t* call = static_cast<call_t*>(inv->mod_data[mod_siprtp_.id]);
  /* Do nothing if media negotiation has failed */
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("SDP negotiation failed");
    return;
  }
  // get stream definition from the SDP, (local contains negotiated data)
  pjmedia_sdp_neg_get_active_local(inv->neg, &local_sdp);
  pjmedia_sdp_neg_get_active_remote(inv->neg, &remote_sdp);
  print_sdp(local_sdp);
  print_sdp(remote_sdp);
  if (call->ice_trans_send_ &&
      !negotiate_ice(call->ice_trans_send_.get(), remote_sdp, inv->dlg->pool))
    SIPPlugin::this_->warning("ice negotiation as sender failed");
  if (call->ice_trans_ && !negotiate_ice(call->ice_trans_.get(), remote_sdp, inv->dlg->pool))
    SIPPlugin::this_->warning("ice negotiation as receiver failed");
  // sending streams
  for (uint i = 0; i < call->media.size(); i++) {
    if (PJCallUtils::is_send_media(local_sdp->media[i])) {
      SIPPlugin::this_->debug(
          "sending data to %",
          std::string(remote_sdp->origin.addr.ptr, remote_sdp->origin.addr.slen));
      auto it = SIPPlugin::this_->sip_calls_->readers_.find(call->media[i].shm_path_to_send);
      if (it == SIPPlugin::this_->sip_calls_->readers_.end()) {
        SIPPlugin::this_->warning("no ShmdataToCb found for sending % (PJCall)",
                                  call->media[i].shm_path_to_send);
      } else {
        // set default address for ICE sending
        pj_sockaddr_init(pj_AF_INET(), &call->media[i].def_addr, NULL, 0);
        pj_sockaddr_set_str_addr(pj_AF_INET(), &call->media[i].def_addr, &remote_sdp->origin.addr);
        pj_sockaddr_set_port(&call->media[i].def_addr,
                             (pj_uint16_t)remote_sdp->media[i]->desc.port);
        auto* def_addr = &call->media[i].def_addr;
        // add a callback for sending when data is available
        call->media[i].cb_id = it->second->add_cb([=](void* data, size_t size) {
          auto comp_id = i + 1;
          SIPPlugin::this_->pjsip_->run([&]() {
            if (!call->ice_trans_send_->sendto(
                    comp_id, data, size, def_addr, pj_sockaddr_get_len(def_addr))) {
              SIPPlugin::this_->debug("issue sending data with ICE");
            }
          });
        });
      }
    }
  }  // end iterating media
}

void PJCall::process_incoming_call(pjsip_rx_data* rdata) {
  if (!SIPPlugin::this_ || !SIPPlugin::this_->sip_calls_.get()) return;
  std::lock_guard<std::mutex> lock(SIPPlugin::this_->sip_calls_->finalize_incoming_calls_m_);
  if (!SIPPlugin::this_ || !SIPPlugin::this_->sip_calls_.get() ||
      !SIPPlugin::this_->sip_calls_->can_create_calls_) {
    SIPPlugin::this_->warning("Trying to initiate a call after all calls are hung out.");
    return;
  }

  // finding caller info
  char uristr[PJSIP_MAX_URL_SIZE];
  int len = pjsip_uri_print(
      PJSIP_URI_IN_REQ_URI, rdata->msg_info.msg->line.req.uri, uristr, sizeof(uristr));
  SIPPlugin::this_->debug("incomimg call from %", std::string(uristr, len));
  len = pjsip_uri_print(PJSIP_URI_IN_FROMTO_HDR,
                        pjsip_uri_get_uri(rdata->msg_info.from->uri),
                        uristr,
                        sizeof(uristr));
  std::string from_uri(uristr, len);
  // find related buddy id ('sip:' is not saved)
  auto peer_uri = std::string(from_uri, 4, std::string::npos);
  auto peer_uri_lower_case = std::string(from_uri, 4, std::string::npos);
  StringUtils::tolower(peer_uri_lower_case);
  if (!SIPPlugin::this_->white_list_->is_authorized(peer_uri) &&
      !SIPPlugin::this_->white_list_->is_authorized(peer_uri_lower_case)) {
    SIPPlugin::this_->message("ERROR:call refused from %", peer_uri);
    SIPPlugin::this_->debug("call refused from %", peer_uri);
    pjsip_endpt_respond_stateless(
        PJSIP::this_->sip_endpt_, rdata, PJSIP_SC_BUSY_HERE, nullptr, nullptr, nullptr);
    return;
  }

  // release existing incoming calls from this buddy
  for (auto& it : SIPPlugin::this_->sip_calls_->incoming_call_) {
    if (it->peer_uri == peer_uri) {
      auto buddy_id = SIPPlugin::this_->sip_presence_->buddy_id_.find(peer_uri);
      if (SIPPlugin::this_->sip_presence_->buddy_id_.end() != buddy_id)
        release_incoming_call(it.get(), buddy_id->second);
    }
  }
  // len =
  // pjsip_uri_print(
  //     PJSIP_URI_IN_FROMTO_HDR, rdata->msg_info.to->uri, uristr,
  //     sizeof(uristr));
  // g_print("----------- call to %.*s", len, uristr);
  pjsip_dialog* dlg;
  pjmedia_sdp_session* sdp;
  pjsip_tx_data* tdata;
  pj_status_t status;
  /* Parse SDP from incoming request and verify that we can handle the
   * request.
   */
  pjmedia_sdp_session* offer = nullptr;
  if (rdata->msg_info.msg->body) {
    pjsip_rdata_sdp_info* sdp_info;
    sdp_info = pjsip_rdata_get_sdp_info(rdata);
    offer = sdp_info->sdp;
    if (nullptr == offer) SIPPlugin::this_->warning("offer is null");
    status = sdp_info->sdp_err;
    if (status == PJ_SUCCESS && sdp_info->sdp == nullptr)
      status = PJSIP_ERRNO_FROM_SIP_STATUS(PJSIP_SC_NOT_ACCEPTABLE);
    if (status != PJ_SUCCESS) SIPPlugin::this_->warning("Bad SDP in incoming INVITE");
  }
  unsigned options = 0;
  status =
      pjsip_inv_verify_request(rdata, &options, nullptr, nullptr, PJSIP::this_->sip_endpt_, &tdata);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("%: can't handle incoming INVITE request", std::string(__FUNCTION__));
    if (tdata) {
      pjsip_response_addr res_addr;
      pjsip_get_response_addr(tdata->pool, rdata, &res_addr);
      pjsip_endpt_send_response(PJSIP::this_->sip_endpt_, &res_addr, tdata, nullptr, nullptr);
    } else { /* Respond with 500 (Internal Server Error) */
      pjsip_endpt_respond_stateless(
          PJSIP::this_->sip_endpt_, rdata, 500, nullptr, nullptr, nullptr);
    }
    return;
  }
  // Create UAS dialog
  status = pjsip_dlg_create_uas_and_inc_lock(pjsip_ua_instance(), rdata, nullptr, &dlg);
  if (status != PJ_SUCCESS) {
    pj_str_t reason;
    pj_cstr(&reason, "Unable to create dialog");
    pjsip_endpt_respond_stateless(PJSIP::this_->sip_endpt_, rdata, 500, &reason, nullptr, nullptr);
    return;
  }
  pjsip_dlg_dec_lock(dlg);
  // we expect the outgoing INVITE to be challenged
  pjsip_auth_clt_set_credentials(
      &dlg->auth_sess, 1, &SIPPlugin::this_->sip_presence_->cfg_.cred_info[0]);
  // incoming call is valid, starting processing it
  SIPPlugin::this_->sip_calls_->incoming_call_.emplace_back(std::make_unique<call_t>());
  call_t* call = SIPPlugin::this_->sip_calls_->incoming_call_.back().get();
  call->peer_uri = peer_uri;
  auto& buddy_list = SIPPlugin::this_->sip_presence_->buddy_id_;
  if (buddy_list.end() == buddy_list.find(call->peer_uri)) {
    SIPPlugin::this_->sip_presence_->add_buddy(call->peer_uri);
    SIPPlugin::this_->sip_presence_->name_buddy(call->peer_uri, call->peer_uri);
  }
  // checking number of transport to create for receiving
  // and creating transport for receiving data offered
  std::vector<pjmedia_sdp_media*> media_to_receive;

  for (unsigned int media_index = 0; media_index < offer->media_count; media_index++) {
    bool recv = false;
    pjmedia_sdp_media* tmp_media = nullptr;
    for (unsigned int j = 0; j < offer->media[media_index]->attr_count; j++) {
      if (0 == pj_strcmp2(&offer->media[media_index]->attr[j]->name, "sendrecv") ||
          0 == pj_strcmp2(&offer->media[media_index]->attr[j]->name, "sendonly")) {
        tmp_media = pjmedia_sdp_media_clone(dlg->pool, offer->media[media_index]);
        pj_cstr(&tmp_media->attr[j]->name, "recvonly");
        recv = true;
      }
    }
    // checking remote ICE candidate(s) and remove them from answer
    if (nullptr != tmp_media) {
      std::vector<pjmedia_sdp_attr*> attr_to_remove;
      for (unsigned int j = 0; j < tmp_media->attr_count; j++) {
        if (0 == pj_strcmp2(&tmp_media->attr[j]->name, "candidate")) {
          attr_to_remove.push_back(tmp_media->attr[j]);
        }
      }
      for (auto& it : attr_to_remove) {
        pjmedia_sdp_media_remove_attr(tmp_media, it);
      }
    }
    // save media for the answer
    if (recv && nullptr != tmp_media) {
      media_to_receive.push_back(pjmedia_sdp_media_clone(dlg->pool, tmp_media));
      call->media.emplace_back();
    }
  }
  call->ice_trans_ = SIPPlugin::this_->stun_turn_->get_ice_transport(media_to_receive.size(),
                                                                     PJ_ICE_SESS_ROLE_CONTROLLED);
  if (!call->ice_trans_) SIPPlugin::this_->warning("ICE transport initialization failed");
  // initializing shmdata writers and linking with ICE transport
  call->recv_rtp_session_ = std::make_unique<gst::RTPSession>();
  for (auto& it : media_to_receive) {
    auto shm_prefix = SIPPlugin::this_->get_shmpath_prefix() +
                      SIPPlugin::this_->get_manager_name() + "_" + SIPPlugin::this_->get_name() +
                      "-" + std::string(call->peer_uri, 0, call->peer_uri.find('@')) + "_";
    auto media_label = PJCallUtils::get_media_label(it);
    auto rtp_shmpath = shm_prefix + "rtp-" + media_label;
    // ensure rtp_shmpath is unique (can happen in case of label collision) and add suffix to
    // media_label if necessary
    unsigned int i = 1;
    std::string rtp_shmpath_suffix;
    while (call->rtp_writers_.end() !=
           std::find_if(call->rtp_writers_.cbegin(),
                        call->rtp_writers_.cend(),
                        [testpath = rtp_shmpath + rtp_shmpath_suffix](
                            const std::unique_ptr<ShmdataWriter>& writer) {
                          return writer->get_path() == testpath;
                        })) {
      ++i;
      rtp_shmpath_suffix = std::to_string(i);
    }
    if (!rtp_shmpath_suffix.empty()) {
      rtp_shmpath = rtp_shmpath + rtp_shmpath_suffix;
      media_label = media_label + rtp_shmpath_suffix;
    }
    // get caps
    auto rtp_caps = PJCallUtils::get_rtp_caps(it);
    if (rtp_caps.empty()) rtp_caps = "unknown_data_type";

    call->rtp_writers_.emplace_back(
        std::make_unique<ShmdataWriter>(SIPPlugin::this_, rtp_shmpath, 1, rtp_caps));
    // uncomment the following in order to get rtp shmdata shown in scenic:
    // SIPPlugin::this_->graft_tree(
    //     std::string(".shmdata.writer.") + rtp_shmpath + ".uri",
    //     InfoTree::make(call->peer_uri));
    auto* writer = call->rtp_writers_.back().get();
    call->ice_trans_->set_data_cb(call->rtp_writers_.size(),
                                  [writer, rtp_shmpath](void* data, size_t size) {
                                    writer->writer<MPtr(&shmdata::Writer::copy_to_shm)>(data, size);
                                    writer->bytes_written(size);
                                  });
    // setting a decoder for this shmdata
    // Create a shmdata quiddity for this stream.
    std::string quid_name =
        media_label + "-" + std::string(call->peer_uri, 0, call->peer_uri.find('@'));
    SIPPlugin::this_->create_quiddity_stream(call->peer_uri, quid_name);
    // Create a gst::RTPReceiver
    call->rtp_receivers_.emplace_back(std::make_unique<gst::RTPReceiver>(
        call->recv_rtp_session_.get(),
        rtp_shmpath,
        [=](GstElement* el, const std::string& media_type, const std::string&) {
          auto shmpath = shm_prefix + media_label + "-" + media_type;
          g_object_set(G_OBJECT(el), "socket-path", shmpath.c_str(), nullptr);
          std::lock_guard<std::mutex> lock(call->shm_subs_mtx_);
          call->shm_subs_.emplace_back(std::make_unique<GstShmTreeUpdater>(
              SIPPlugin::this_,
              el,
              shmpath,
              GstShmTreeUpdater::Direction::writer,
              [=](const std::string& /*caps*/) {
                SIPPlugin::this_->graft_tree(std::string(".shmdata.writer.") + shmpath + ".uri",
                                             InfoTree::make(call->peer_uri));
                // Connect the stream to the created shmdata quiddity.
                SIPPlugin::this_->expose_stream_to_quiddity(quid_name, shmpath);
              },
              [=, uri_to_remove = call->peer_uri]() {
                SIPPlugin::this_->remove_exposed_quiddity(uri_to_remove, quid_name);
              }));
        },
        SIPPlugin::this_->decompress_streams_));
  }
  // Create SDP answer
  create_sdp_answer(dlg->pool, call, media_to_receive, &sdp);
  // preparing initial answer
  int initial_answer_code = 200;
  // Create UAS invite session
  status = pjsip_inv_create_uas(dlg, rdata, sdp, 0, &call->inv);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->debug("error creating uas");
    pjsip_dlg_create_response(dlg, rdata, 500, nullptr, &tdata);
    pjsip_dlg_send_response(dlg, pjsip_rdata_get_tsx(rdata), tdata);
    return;
  }
  /* Attach call data to invite session */
  call->inv->mod_data[mod_siprtp_.id] = call;
  /* Create response . */
  status =
      pjsip_inv_initial_answer(call->inv, rdata, initial_answer_code, nullptr, nullptr, &tdata);
  if (status != PJ_SUCCESS) {
    status = pjsip_inv_initial_answer(
        call->inv, rdata, PJSIP_SC_NOT_ACCEPTABLE, nullptr, nullptr, &tdata);
    if (status == PJ_SUCCESS)
      pjsip_inv_send_msg(call->inv, tdata);
    else
      pjsip_inv_terminate(call->inv, 500, PJ_FALSE);
    return;
  }
  /* Send the initial response. */
  status = pjsip_inv_send_msg(call->inv, tdata);
  if (PJ_SUCCESS != status) {
    SIPPlugin::this_->warning("cannot answer to a call, it probably has too many streams");
    pjsip_response_addr res_addr;
    pjsip_get_response_addr(tdata->pool, rdata, &res_addr);
    pjsip_endpt_send_response(PJSIP::this_->sip_endpt_, &res_addr, tdata, nullptr, nullptr);
  }
}

pj_status_t PJCall::create_sdp_answer(pj_pool_t* pool,
                                      call_t* call,
                                      const std::vector<pjmedia_sdp_media*>& media_to_receive,
                                      pjmedia_sdp_session** p_sdp) {
  PJ_ASSERT_RETURN(pool && p_sdp, PJ_EINVAL);
  /* Create and initialize basic SDP session */
  pjmedia_sdp_session* sdp =
      static_cast<pjmedia_sdp_session*>(pj_pool_zalloc(pool, sizeof(pjmedia_sdp_session)));
  pj_time_val tv;
  pj_gettimeofday(&tv);
  pj_cstr(&sdp->origin.user, "-");
  sdp->origin.version = sdp->origin.id = tv.sec + 2208988800UL;
  pj_cstr(&sdp->origin.net_type, "IN");
  pj_cstr(&sdp->origin.addr_type, "IP4");
  sdp->origin.addr = SIPPlugin::this_->sip_calls_->local_addr_;
  pj_cstr(&sdp->name, "switcher");
  sdp->conn = static_cast<pjmedia_sdp_conn*>(pj_pool_zalloc(pool, sizeof(pjmedia_sdp_conn)));
  pj_cstr(&sdp->conn->net_type, "IN");
  pj_cstr(&sdp->conn->addr_type, "IP4");
  sdp->conn->addr = SIPPlugin::this_->sip_calls_->local_addr_;
  /* SDP time and attributes. */
  sdp->time.start = sdp->time.stop = 0;
  sdp->attr_count = 0;
  if (call->ice_trans_) {
    auto ufrag_pwd = call->ice_trans_->get_ufrag_and_passwd();
    pjmedia_sdp_attr* ufrag =
        static_cast<pjmedia_sdp_attr*>(pj_pool_zalloc(pool, sizeof(pjmedia_sdp_attr)));
    ufrag->name = pj_str((char*)"ice-ufrag");
    ufrag->value = ufrag_pwd.first;
    sdp->attr[sdp->attr_count] = ufrag;
    ++sdp->attr_count;
    pjmedia_sdp_attr* pwd =
        static_cast<pjmedia_sdp_attr*>(pj_pool_zalloc(pool, sizeof(pjmedia_sdp_attr)));
    pwd->name = pj_str((char*)"ice-pwd");
    pwd->value = ufrag_pwd.second;
    sdp->attr[sdp->attr_count] = pwd;
    ++sdp->attr_count;
  }
  sdp->media_count = 0;
  auto candidates = call->ice_trans_->get_components();
  auto default_ports = call->ice_trans_->get_first_candidate_ports();
  for (unsigned i = 0; i < media_to_receive.size(); i++) {
    // getting offer media to receive
    pjmedia_sdp_media* sdp_media = media_to_receive[i];
    // set port
    sdp_media->desc.port = default_ports[i];
    for (auto& it : candidates[i]) {
      pjmedia_sdp_attr* cand =
          static_cast<pjmedia_sdp_attr*>(pj_pool_zalloc(pool, sizeof(pjmedia_sdp_attr)));
      cand->name = pj_str((char*)"candidate");
      cand->value = pj_strdup3(pool, it.c_str());
      sdp_media->attr[sdp_media->attr_count] = cand;
      ++sdp_media->attr_count;
    }
    // put media in answer
    sdp->media[i] = sdp_media;
    sdp->media_count++;
  }
  *p_sdp = sdp;
  return PJ_SUCCESS;
}

void PJCall::print_sdp(const pjmedia_sdp_session* local_sdp) {
  char sdpbuf1[65536];
  pj_ssize_t len1;
  len1 = pjmedia_sdp_print(local_sdp, sdpbuf1, sizeof(sdpbuf1));
  if (len1 < 1) {
    SIPPlugin::this_->warning("error when printing local sdp");
    return;
  }
  sdpbuf1[len1] = '\0';
  SIPPlugin::this_->debug("sdp : \n% \n ", std::string(sdpbuf1));
}

bool PJCall::make_call(std::string dst_uri) {
  if (SIPPlugin::this_->sip_presence_->sip_local_user_.empty()) {
    SIPPlugin::this_->warning("cannot call if not registered");
    return false;
  }
  auto it = std::find_if(
      outgoing_call_.begin(), outgoing_call_.end(), [&dst_uri](const std::unique_ptr<call_t>& c) {
        return c->peer_uri == dst_uri;
      });
  if (it != outgoing_call_.end()) {
    SIPPlugin::this_->warning("cannot call % (already calling)", dst_uri);
    return false;
  }
  auto& sip_local_user = SIPPlugin::this_->sip_presence_->sip_local_user_;
  if (std::string("sip:") + dst_uri ==
      std::string(sip_local_user, 0, sip_local_user.find_last_of(':'))) {
    SIPPlugin::this_->message("ERROR:cannot self call");
    SIPPlugin::this_->warning("cannot self call");
    return false;
  }

  pj_str_t local_uri;
  std::string local_uri_tmp(sip_local_user);
  pj_cstr(&local_uri, local_uri_tmp.c_str());
  call_t* cur_call = nullptr;
  pjsip_dialog* dlg = nullptr;
  pjmedia_sdp_session* sdp = nullptr;
  pjsip_tx_data* tdata = nullptr;
  pj_status_t status;
  pj_str_t dest_str;
  std::string tmp_dest_uri("sip:" + dst_uri);
  pj_cstr(&dest_str, tmp_dest_uri.c_str());
  auto id = pjsua_buddy_find(&dest_str);
  if (PJSUA_INVALID_ID == id) {
    SIPPlugin::this_->warning("buddy not found: cannot call %", dst_uri);
    return false;
  }
  auto paths = SIPPlugin::this_->tree<MPtr(&InfoTree::copy_leaf_values)>(
      std::string(".buddies." + std::to_string(id) + ".connections"));
  if (paths.empty()) {
    SIPPlugin::this_->warning("not calling % since no shmdata path has been attached", dst_uri);
    return false;
  }
  // Find unused call slot
  outgoing_call_.emplace_back(std::make_unique<call_t>());
  cur_call = outgoing_call_.back().get();
  // Create UAC dialog
  status = pjsip_dlg_create_uac(pjsip_ua_instance(),
                                &local_uri, /* local URI */
                                nullptr,    /* local Contact */
                                &dest_str,  /* remote URI */
                                nullptr,    /* remote target */
                                &dlg);      /* dialog */
  if (status != PJ_SUCCESS) {
    char errstr[1024];
    pj_strerror(status, errstr, 1024);
    SIPPlugin::this_->warning("pjsip_dlg_create_uac FAILLED %", std::string(errstr));
    return false;
  }
  /* we expect the outgoing INVITE to be challenged*/
  pjsip_auth_clt_set_credentials(
      &dlg->auth_sess, 1, &SIPPlugin::this_->sip_presence_->cfg_.cred_info[0]);
  cur_call->peer_uri = dst_uri;
  // Create SDP
  if (!create_outgoing_sdp(dlg, cur_call, &sdp)) {
    SIPPlugin::this_->warning("% failed creating sdp", std::string(__FUNCTION__));
    pjsip_dlg_terminate(dlg);
    outgoing_call_.pop_back();
    return false;
  }
  print_sdp(sdp);
  // Create the INVITE session.
  status = pjsip_inv_create_uac(dlg, sdp, 0, &cur_call->inv);
  if (status != PJ_SUCCESS) {
    pjsip_dlg_terminate(dlg);
    SIPPlugin::this_->warning("pjsip_inv_create_uac FAILLED");
    outgoing_call_.pop_back();
    return false;
  }
  // Attach call data to invite session
  cur_call->inv->mod_data[mod_siprtp_.id] = cur_call;
  /* Create initial INVITE request.
   * This INVITE request will contain a perfectly good request and
   * an SDP body as well.
   */
  status = pjsip_inv_invite(cur_call->inv, &tdata);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("pjsip_inv_invite error");
    return false;
  }
  /* Send initial INVITE request.
   * From now on, the invite session's state will be reported to us
   * via the invite session callbacks.
   */
  status = pjsip_inv_send_msg(cur_call->inv, tdata);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("pjsip_inv_send_msg error");
    return false;
  }
  // updating call status in the tree
  InfoTree::ptr tree =
      SIPPlugin::this_->prune_tree(std::string(".buddies." + std::to_string(id)),
                                   false);  // do not signal since the branch will be re-grafted
  if (!tree) {
    SIPPlugin::this_->warning("cannot find buddy information tree, call cancelled");
    return false;
  }
  tree->graft(std::string(".send_status."), InfoTree::make("calling"));
  SIPPlugin::this_->graft_tree(std::string(".buddies." + std::to_string(id)), tree);
  return true;
}

bool PJCall::create_outgoing_sdp(pjsip_dialog* dlg, call_t* call, pjmedia_sdp_session** res) {
  pj_str_t contact;
  std::string tmpstr("sip:" + call->peer_uri);
  pj_cstr(&contact, tmpstr.c_str());
  auto id = pjsua_buddy_find(&contact);
  if (PJSUA_INVALID_ID == id) {
    SIPPlugin::this_->warning("buddy not found: cannot call %", call->peer_uri);
    return false;
  }
  auto paths = SIPPlugin::this_->tree<MPtr(&InfoTree::copy_leaf_values)>(
      std::string(".buddies." + std::to_string(id) + ".connections"));
  if (paths.empty()) return false;
  // creating ice transport for sending
  call->ice_trans_send_ =
      SIPPlugin::this_->stun_turn_->get_ice_transport(paths.size(), PJ_ICE_SESS_ROLE_CONTROLLING);
  if (!call->ice_trans_send_) {
    SIPPlugin::this_->warning("cannot init ICE transport for sending");
    return false;
  }
  // making SDP description
  gst::SDPDescription desc(call->ice_trans_send_->get_first_candidate_host());
  // adding ICE ufrag and pwd to sdp session
  auto ufrag_pwd = call->ice_trans_send_->get_ufrag_and_passwd();
  if (!desc.add_msg_attribute("ice-ufrag", std::string(ufrag_pwd.first.ptr, ufrag_pwd.first.slen)))
    SIPPlugin::this_->warning("issue adding ice-ufrag");
  if (!desc.add_msg_attribute("ice-pwd", std::string(ufrag_pwd.second.ptr, ufrag_pwd.second.slen)))
    SIPPlugin::this_->warning("issue adding ice-pwd");
  // adding media and candidate lines to each the media
  auto default_ports = call->ice_trans_send_->get_first_candidate_ports();
  auto candidates = call->ice_trans_send_->get_components();
  for (auto& it : paths) {
    std::string rtpcaps;
    auto reader = readers_.find(it);
    if (reader != readers_.cend() && reader->second) {
      rtpcaps = reader->second->get_caps();
    } else {
      break;
    }

    // Shmdata received from SIP have a very different nomenclature than those created locally.
    // When sending those shmdatas received from SIP to someone else (i.e. creating a relay),
    // we mustn't use the get_quiddity_name_from_file_name() function; rather, we must interrogate
    // the SIP quiddity (which manages the shmpaths received from SIP contacts).
    std::string rawlabel = SIPPlugin::this_->get_exposed_quiddity_name_from_shmpath(it);
    if (rawlabel.empty()) {
      rawlabel = SIPPlugin::this_->get_quiddity_name_from_file_name(it);
    }
    auto quid_id = SIPPlugin::this_->qcontainer_->get_id(rawlabel);
    auto quid = SIPPlugin::this_->qcontainer_->get_quiddity(quid_id);
    if (quid) rawlabel = quid->get_nickname();
    std::istringstream ss(rawlabel);  // Turn the string into a stream
    std::string tok;
    std::getline(ss, tok, ' ');
    std::string label = tok;
    while (std::getline(ss, tok, ' ')) label += tok;
    rtpcaps += ", media-label=(string)\"" +
               StringUtils::replace_char(StringUtils::base64_encode(label), '=', "") + "\"";
    GstCaps* caps = gst_caps_from_string(rtpcaps.c_str());
    On_scope_exit { gst_caps_unref(caps); };
    gst::SDPMedia media;
    media.set_media_info_from_caps(caps);
    media.set_port(default_ports.back());
    for (auto& it : candidates.back()) {
      media.add_ice_candidate(it);
    }
    default_ports.pop_back();
    candidates.pop_back();
    if (!desc.add_media(media)) {
      SIPPlugin::this_->warning("a media has not been added to the SDP description");
    } else {
      call->media.emplace_back();
      call->media.back().shm_path_to_send = it;
    }
  }
  if (call->media.empty()) {
    SIPPlugin::this_->warning("no valid media stream found, aborting call");
    return false;
  }
  // checking produced SDP
  std::string desc_str = desc.get_string();
  if (desc_str.empty()) {
    SIPPlugin::this_->warning("%: empty SDP description", std::string(__FUNCTION__));
    return false;
  }
  pj_str_t sdp_str;
  pj_strdup2(dlg->pool, &sdp_str, desc_str.c_str());
  pj_status_t status = pjmedia_sdp_parse(dlg->pool, sdp_str.ptr, sdp_str.slen, res);
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("pjmedia_sdp_parse FAILLED in %", std::string(__FUNCTION__));
    return false;
  }
  return true;
}

bool PJCall::send_to(const std::string& sip_url) {
  std::lock_guard<std::mutex> lock(finalize_outgoing_calls_m_);

  if (!can_create_calls_) {
    SIPPlugin::this_->warning("Trying to initiate a call after all calls have been hung out.");
    return false;
  }
  if (sip_url.empty()) {
    SIPPlugin::this_->warning("calling sip account received nullptr url");
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(call_m_, std::defer_lock);
    if (!lock.try_lock()) {
      SIPPlugin::this_->debug("cancel SIP send_to because an operation is already pending");
      return false;
    }
    is_calling_ = true;
    On_scope_exit { is_calling_ = false; };
    auto res =
        SIPPlugin::this_->pjsip_->run<bool>([&]() { return make_call(std::string(sip_url)); });
    if (res) {
      call_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {
        if (call_action_done_) {
          call_action_done_ = false;
          return true;
        }
        return false;
      });
    }
  }
  return true;
}

bool PJCall::hang_up(const std::string& sip_url) {
  if (sip_url.empty()) {
    SIPPlugin::this_->warning("hang up received nullptr url");
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(call_m_, std::defer_lock);
    if (!lock.try_lock()) {
      SIPPlugin::this_->debug("cancel SIP hang_up because an operation is already pending");
      return false;
    }
    is_hanging_up_ = true;
    On_scope_exit { is_hanging_up_ = false; };

    auto it_out = std::find_if(outgoing_call_.begin(),
                               outgoing_call_.end(),
                               [&sip_url](const std::unique_ptr<call_t>& call) {
                                 return (std::string(sip_url) == call->peer_uri);
                               });

    if (it_out != outgoing_call_.end()) {
      auto invite_out = (*it_out)->inv;
      SIPPlugin::this_->pjsip_->run_async([this, &invite_out]() { make_hang_up(invite_out); });
      call_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {
        if (call_action_done_) {
          call_action_done_ = false;
          return true;
        }
        return false;
      });

      // stop here, do not hangup incoming call.
      return true;
    }

    auto it_inc = std::find_if(incoming_call_.begin(),
                               incoming_call_.end(),
                               [&sip_url](const std::unique_ptr<call_t>& call) {
                                 return (std::string(sip_url) == call->peer_uri);
                               });

    if (it_inc != incoming_call_.end()) {
      auto invite_inc = (*it_inc)->inv;
      SIPPlugin::this_->pjsip_->run_async([this, &invite_inc]() { make_hang_up(invite_inc); });
      call_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {
        if (call_action_done_) {
          call_action_done_ = false;
          return true;
        }
        return false;
      });

      return true;
    }
  }
  return false;
}

void PJCall::make_hang_up(pjsip_inv_session* inv) {
  pjsip_tx_data* tdata;
  pj_status_t status;

  status = pjsip_inv_end_session(inv, 603, nullptr, &tdata);

  if (status == PJ_SUCCESS && tdata != nullptr)
    pjsip_inv_send_msg(inv, tdata);
  else
    SIPPlugin::this_->warning("BYE has not been sent");
}

bool PJCall::attach_shmdata_to_contact(const std::string& shmpath,
                                       const std::string& contact_uri,
                                       bool attach) {
  if (shmpath.empty() || contact_uri.empty()) {
    SIPPlugin::this_->warning("cannot add shmpath for user (received nullptr)");
    return false;
  }
  SIPPlugin::this_->pjsip_->run([&]() {
    make_attach_shmdata_to_contact(std::string(shmpath), std::string(contact_uri), attach);
  });
  return true;
}

void PJCall::make_attach_shmdata_to_contact(const std::string& shmpath,
                                            const std::string& contact_uri,
                                            bool attach) {
  auto& sip_local_user = SIPPlugin::this_->sip_presence_->sip_local_user_;
  if (std::string("sip:") + contact_uri ==
      std::string(sip_local_user, 0, sip_local_user.find_last_of(':'))) {
    SIPPlugin::this_->message("ERROR:cannot attach shmdata to self");
    SIPPlugin::this_->warning("cannot attach shmdata to self");
    return;
  }

  pj_str_t contact;
  std::string tmpstr("sip:" + contact_uri);
  pj_cstr(&contact, tmpstr.c_str());
  auto id = pjsua_buddy_find(&contact);
  if (PJSUA_INVALID_ID == id) {
    SIPPlugin::this_->warning("buddy not found: cannot attach % to %", shmpath, contact_uri);
    return;
  }
  if (attach) {
    InfoTree::ptr tree =
        SIPPlugin::this_->prune_tree(std::string(".buddies." + std::to_string(id)),
                                     false);  // do not signal since the branch will be re-grafted
    if (!tree) tree = InfoTree::make();
    if (readers_.find(shmpath) == readers_.cend()) {
      readers_.emplace(shmpath, std::make_unique<gst::RTPSender>(&rtp_session_, shmpath, 1400));
      reader_ref_count_[shmpath] = 1;
    } else {
      ++reader_ref_count_[shmpath];
    }
    tree->graft(std::string(".connections." + shmpath), InfoTree::make(shmpath));
    tree->tag_as_array(".connections", true);
    SIPPlugin::this_->graft_tree(".buddies." + std::to_string(id), tree);
    return;
  }
  // detach
  auto it = readers_.find(shmpath);
  if (it == readers_.end()) {
    SIPPlugin::this_->warning("error detaching a shmdata not attached (PJCall)");
    return;
  }
  auto remaining = --reader_ref_count_[shmpath];
  if (0 == remaining) {
    readers_.erase(it);
    reader_ref_count_.erase(shmpath);
  }
  auto tree =
      SIPPlugin::this_->prune_tree(".buddies." + std::to_string(id) + ".connections", false);
  tree->prune(shmpath);
  SIPPlugin::this_->graft_tree(".buddies." + std::to_string(id) + ".connections", tree);
}

void PJCall::call_on_rx_offer(pjsip_inv_session* /*inv*/, const pjmedia_sdp_session* /*offer*/) {
  // print_sdp(offer);
}

bool PJCall::negotiate_ice(PJICEStreamTrans* ice_trans,
                           const pjmedia_sdp_session* remote_sdp,
                           pj_pool_t* dlg_pool) {
  // checking ICE
  pjmedia_sdp_attr* ufrag =
      pjmedia_sdp_attr_find2(remote_sdp->attr_count, remote_sdp->attr, "ice-ufrag", nullptr);
  if (nullptr == ufrag) return false;
  SIPPlugin::this_->debug("ICE ufrag received: %",
                          std::string(ufrag->value.ptr, 0, ufrag->value.slen));
  pjmedia_sdp_attr* pwd =
      pjmedia_sdp_attr_find2(remote_sdp->attr_count, remote_sdp->attr, "ice-pwd", nullptr);
  if (nullptr == pwd) return false;
  SIPPlugin::this_->debug("ICE pwd received: %", std::string(pwd->value.ptr, 0, pwd->value.slen));
  // candidates
  unsigned cand_cnt = 0;
  pj_ice_sess_cand candidates[PJ_ICE_ST_MAX_CAND];
  std::vector<unsigned> sending_medias(remote_sdp->media_count, 0);
  for (unsigned i = 0; i < remote_sdp->media_count; ++i) {
    for (unsigned j = 0; j < remote_sdp->media[i]->attr_count; ++j) {
      if ("candidate" == std::string(remote_sdp->media[i]->attr[j]->name.ptr,
                                     0,
                                     remote_sdp->media[i]->attr[j]->name.slen)) {
        sending_medias[i] = 1;
        pj_ice_sess_cand* cand = &candidates[cand_cnt];
        pj_bzero(cand, sizeof(pj_ice_sess_cand));
        ++cand_cnt;
        SIPPlugin::this_->debug("ICE candidate received: %",
                                std::string(remote_sdp->media[i]->attr[j]->value.ptr,
                                            0,
                                            remote_sdp->media[i]->attr[j]->value.slen));
        int af;
        char foundation[32], transport[12], ipaddr[80], type[32];
        pj_str_t tmpaddr;
        int comp_id, prio, port;
        int cnt = sscanf(std::string(remote_sdp->media[i]->attr[j]->value.ptr,
                                     0,
                                     remote_sdp->media[i]->attr[j]->value.slen)
                             .c_str(),
                         "%s %d %s %d %s %d typ %s",
                         foundation,
                         &comp_id,
                         transport,
                         &prio,
                         ipaddr,
                         &port,
                         type);
        if (cnt != 7) {
          SIPPlugin::this_->warning("error: Invalid ICE candidate line");
          return false;
        }
        if (strcmp(type, "host") == 0)
          cand->type = PJ_ICE_CAND_TYPE_HOST;
        else if (strcmp(type, "srflx") == 0)
          cand->type = PJ_ICE_CAND_TYPE_SRFLX;
        else if (strcmp(type, "relay") == 0)
          cand->type = PJ_ICE_CAND_TYPE_RELAYED;
        else {
          SIPPlugin::this_->warning("Error: invalid candidate type '%'", std::string(type));
          return false;
        }
        cand->comp_id = (pj_uint8_t)comp_id;
        pj_strdup2(dlg_pool, &cand->foundation, foundation);
        cand->prio = prio;
        if (strchr(ipaddr, ':'))
          af = pj_AF_INET6();
        else
          af = pj_AF_INET();
        tmpaddr = pj_str(ipaddr);
        pj_sockaddr_init(af, &cand->addr, NULL, 0);
        if (PJ_SUCCESS != pj_sockaddr_set_str_addr(af, &cand->addr, &tmpaddr)) {
          SIPPlugin::this_->warning("Error: invalid IP address '%'", std::string(ipaddr));
        }
        pj_sockaddr_set_port(&cand->addr, (pj_uint16_t)port);
      }
    }
  }
  if (0 == cand_cnt) return false;

  if (!ice_trans->start_nego(&ufrag->value, &pwd->value, cand_cnt, candidates)) {
    SIPPlugin::this_->warning("Error starting ICE negotiation");
    return false;
  }
  return true;
}

}  // namespace quiddities
}  // namespace switcher
