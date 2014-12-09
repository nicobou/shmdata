/*
 * This file is part of switcher-pjsip.
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

#include <string>
#include <list>
#include "switcher/scope-exit.hpp"
#include "switcher/information-tree-basic-serializer.hpp"
#include "./pj-presence.hpp"
#include "./pj-sip.hpp"

namespace switcher {
GEnumValue PJPresence::status_enum_[5] = {
  {PJPresence::AVAILABLE, "Available", "AVAILABLE"},
  {PJPresence::BUSY, "Busy", "BUSY"},
  {PJPresence::AWAY, "Away", "AWAY"},
  {PJPresence::OFFLINE, "Offline", "OFFLINE"},
  {0, nullptr, nullptr}
};

PJPresence::PJPresence(PJSIP *sip_instance):
    sip_instance_(sip_instance),
    status_(PJPresence::OFFLINE) {
  // registering account
  sip_instance_->
      install_method("Register SIP Account",  // long name
                     "register",  // name
                     "register a SIP account",  // description
                     "the registration has been asked",  // return description
                     Method::make_arg_description("SIP address",  // long name
                                                  "user address",  // name
                                                  "string",  // description
                                                  "SIP password",
                                                  "password",
                                                  "string",
                                                  nullptr),
                     (Method::method_ptr)&register_account_wrapped,
                     G_TYPE_BOOLEAN,
                     Method::make_arg_type_description(G_TYPE_STRING,
                                                       G_TYPE_STRING,
                                                       nullptr),
                     this);
  
  sip_instance_->
      install_method("Unregister SIP Account",  // long name
                     "unregister",  // name
                     "unregister SIP account",  // description
                     "success",  // return description
                     Method::make_arg_description("none",
                                                  nullptr),
                     (Method::method_ptr) &unregister_account_wrapped,
                     G_TYPE_BOOLEAN,
                     Method::make_arg_type_description(G_TYPE_NONE,
                                                       nullptr),
                     this);

  //buddies
    sip_instance_->
      install_method("Add Buddy",  // long name
                     "add_buddy",  // name
                     "add a buddy",  // description
                     "success",  // return description
                     Method::make_arg_description("SIP User Name",  // long name
                                                  "user",  // name
                                                  "string",  // description
                                                  nullptr),
                     (Method::method_ptr)&add_buddy_wrapped,
                     G_TYPE_BOOLEAN,
                     Method::make_arg_type_description(G_TYPE_STRING,
                                                       nullptr),
                     this);
    sip_instance_->
      install_method("Set Buddy Name",  // long name
                     "name_buddy",  // name
                     "give a name to a buddy",  // description
                     "success",  // return description
                     Method::make_arg_description("Name",  // long name
                                                  "name",  // name
                                                  "string",  // description
                                                  "SIP URI",  // long name
                                                  "uri",  // name
                                                  "string",  // description
                                                  nullptr),
                     (Method::method_ptr)&name_buddy_wrapped,
                     G_TYPE_BOOLEAN,
                     Method::make_arg_type_description(G_TYPE_STRING,
                                                       G_TYPE_STRING,
                                                       nullptr),
                     this);
    sip_instance_->
      install_method("Del Buddy",  // long name
                     "del_buddy",  // name
                     "Delete a buddy",  // description
                     "success",  // return description
                     Method::make_arg_description("SIP User Name",  // long name
                                                  "user",  // name
                                                  "string",  // description
                                                  nullptr),
                     (Method::method_ptr)&del_buddy_wrapped,
                     G_TYPE_BOOLEAN,
                     Method::make_arg_type_description(G_TYPE_STRING,
                                                       nullptr),
                     this);

    // FIXME remove or implement
    // sip_instance_->
    //     install_method("Save Buddies",  // long name
    //                    "save_buddies",  // name
    //                    "save buddy informations",  // description
    //                    "success",  // return description
    //                    Method::make_arg_description("File Name",  // long name
    //                                                 "file",  // name
    //                                                 "string",  // description
    //                                                 nullptr),
    //                    (Method::method_ptr)&save_buddies_wrapped,
    //                    G_TYPE_BOOLEAN,
    //                    Method::make_arg_type_description(G_TYPE_STRING,
    //                                                      nullptr),
    //                    this);

    
    // online status
    status_enum_spec_ =
        sip_instance_->custom_props_->
        make_enum_property("status",
                           "Possible Status",
                           status_,
                           status_enum_,
                           (GParamFlags)
                           G_PARAM_READWRITE,
                           PJPresence::set_status,
                           PJPresence::get_status,
                           this);
    sip_instance_->
        install_property_by_pspec(sip_instance_->custom_props_->
                                  get_gobject(),
                                  status_enum_spec_,
                                  "status",
                                  "Online Status");
    custom_status_spec_ = sip_instance_->custom_props_->
        make_string_property("status-note",
                             "Custom status note",
                             "",
                             (GParamFlags)G_PARAM_READWRITE,
                             PJPresence::set_note,
                             PJPresence::get_note,
                             this);
    sip_instance_->
        install_property_by_pspec(sip_instance_->custom_props_->get_gobject(),
                                  custom_status_spec_,
                                  "status-note",
                                  "Custom status note");

    sip_reg_status_spec_ = sip_instance_->custom_props_->
        make_boolean_property("sip-registration",
                              "Self SIP registration status",
                              FALSE,
                              (GParamFlags)G_PARAM_READABLE,
                             nullptr,
                             PJPresence::get_sip_registration_status,
                             this);
    sip_instance_->
        install_property_by_pspec(sip_instance_->custom_props_->get_gobject(),
                                  sip_reg_status_spec_,
                                  "sip-registration",
                                  "Self SIP registration status");

}

PJPresence::~PJPresence() {
  unregister_account();
}

gboolean
PJPresence::register_account_wrapped(gchar *user,
                                     gchar *password,
                                     void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  if (nullptr == user || nullptr == password) {
    g_warning("register sip account missing user or domain or password");
    return FALSE;
  }
  context->sip_instance_->run_command_sync(std::bind
                                           (&PJPresence::register_account,
                                            context,
                                            std::string(user),
                                            std::string(password)));
  return TRUE;
}

void
PJPresence::register_account(const std::string &sip_user,
                             const std::string &sip_password) {
  std::unique_lock<std::mutex> lock(registration_mutex_);
  
  // Register to SIP server by creating SIP account.
  pjsua_acc_config cfg;
  pj_status_t status;

  // extracting domain information from sip uri
  auto at = std::find(sip_user.begin(), sip_user.end(), '@');
  if(sip_user.end() == at || sip_user.end() == (at + 1)) {
    g_warning("%s: invalid sip uri", __FUNCTION__);
    return;
  }

  // converting to gchar * in order to make pjsip happy
  gchar *domain = g_strdup(std::string(at + 1, sip_user.end()).c_str());
  On_scope_exit{g_free(domain);};
  //gchar *id = g_strdup_printf("sip:%s", sip_user.c_str());
  gchar *id = g_strdup_printf("sip:%s;transport=tcp", sip_user.c_str());
  On_scope_exit{g_free(id);};
  gchar *user = g_strdup(std::string(sip_user.begin(), at).c_str());
  On_scope_exit{g_free(user);};
  gchar *digest = g_strdup("digest");
  On_scope_exit{g_free(digest);};
  gchar *password = g_strdup(sip_password.c_str());
  On_scope_exit{g_free(password);};

  // setting pjsip account data structure
  pjsua_acc_config_default(&cfg);
  cfg.id = pj_str(id);
  cfg.reg_uri = pj_str(id);
  cfg.cred_count = 1;
  cfg.cred_info[0].realm = pj_str(domain);
  cfg.cred_info[0].scheme = pj_str(digest);
  cfg.cred_info[0].username = pj_str(user);
  cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
  cfg.cred_info[0].data = pj_str(password);
  cfg.publish_enabled = PJ_TRUE;
  // pjsua_acc_set_registration (account_id_, PJ_TRUE) or:
  cfg.register_on_acc_add = PJ_TRUE;
  
  status = pjsua_acc_add(&cfg, PJ_TRUE, &account_id_);
  if (status != PJ_SUCCESS) {
    account_id_ = -1;
    return;
  }
  pjsua_acc_set_user_data(account_id_, this);
  registration_cond_.wait(lock);
  // notifying sip registration status
  GObjectWrapper::notify_property_changed(sip_instance_->gobject_->get_gobject(),
                                          sip_reg_status_spec_);
  sip_local_user_ = std::string("sip:") + sip_user +
      + ":" + std::to_string(sip_instance_->sip_port_);
}

gboolean PJPresence::unregister_account_wrapped(gpointer /*unused */ ,
                                                void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  context->sip_instance_->
      run_command_sync(std::bind(&PJPresence::unregister_account,
                                 context));
  if (-1 != context->account_id_)
    return FALSE;
  return TRUE;
}

void PJPresence::unregister_account() {
  std::unique_lock<std::mutex> lock(registration_mutex_);
  if (-1 == account_id_) {
    g_warning("no account to unregister");
    return;
  }
  change_online_status(PJPresence::OFFLINE);
  if (PJ_SUCCESS != pjsua_acc_del(account_id_)) {
    g_warning("error when unregistering account");
    return;
  }
  account_id_ = -1;
  sip_local_user_.clear();
  return;
}

void PJPresence::add_buddy(const std::string &sip_user) {
  pjsua_buddy_config buddy_cfg;
  pjsua_buddy_id buddy_id;
  pj_status_t status = PJ_SUCCESS;

  std::string buddy_full_uri("sip:" + sip_user + ";transport=tcp");
  if (pjsua_verify_url(buddy_full_uri.c_str()) != PJ_SUCCESS) {
    g_warning("Invalid buddy URI (%s)", sip_user.c_str());
    return;
  }
  if (buddy_id_.end() != buddy_id_.find(sip_user)) {
    g_message("buddy %s already added", sip_user.c_str());
    return;
  }

  pj_bzero(&buddy_cfg, sizeof(pjsua_buddy_config));
  pj_cstr(&buddy_cfg.uri, buddy_full_uri.c_str());
  buddy_cfg.subscribe = PJ_TRUE;
  buddy_cfg.user_data = this;
  status = pjsua_buddy_add(&buddy_cfg, &buddy_id);
  if (status != PJ_SUCCESS) {
    g_warning("buddy not found");
    return;
  }
  g_debug("Buddy added");
  buddy_id_[sip_user] = buddy_id;
  sip_instance_->
      graft_tree(".buddy." + std::to_string(buddy_id) + ".uri",
                 data::Tree::make(sip_user));
  sip_instance_->
      graft_tree(".buddy." + std::to_string(buddy_id) + ".call_status",
                 data::Tree::make("disconnected"));
  return;
}

void PJPresence::del_buddy(const std::string &sip_user) {
  pj_status_t status = PJ_SUCCESS;

  if (pjsua_verify_url(std::string("sip:" + sip_user).c_str()) != PJ_SUCCESS) {
    g_warning("Invalid buddy URI (%s) for deletion", sip_user.c_str());
    return;
  }
  auto it = buddy_id_.find(sip_user);
  if (buddy_id_.end() != it) {
    g_debug("%s is not in buddy list, cannot delete", sip_user.c_str());
    return;
  }

  status = pjsua_buddy_del(it->second);
  if (status != PJ_SUCCESS) {
    g_warning("cannot remove buddy");
    return;
  }
  sip_instance_->prune_tree("buddy."+ std::to_string(it->second));
  buddy_id_.erase(it);
  g_debug("Buddy removed");
  return;
}

gboolean PJPresence::add_buddy_wrapped(gchar *buddy_uri,
                                       void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  context->sip_instance_->
      run_command_sync(std::bind(&PJPresence::add_buddy,
                                 context,
                                 buddy_uri));
  return TRUE;
}

gboolean PJPresence::del_buddy_wrapped(gchar *buddy_uri,
                                       void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  context->sip_instance_->
      run_command_sync(std::bind(&PJPresence::del_buddy,
                                 context,
                                 buddy_uri));
  return TRUE;
}

void PJPresence::name_buddy(std::string name, std::string sip_user) {
  
  if (pjsua_verify_url(std::string("sip:" + sip_user).c_str()) != PJ_SUCCESS) {
    g_warning("Invalid buddy URI (%s) when giving a nick name", sip_user.c_str());
    return;
  }
  auto it = buddy_id_.find(sip_user);
  if (buddy_id_.end() == it) {
    g_warning("%s is not in buddy list", sip_user.c_str());
    return;
  }
  sip_instance_->
      graft_tree(".buddy." + std::to_string(it->second) + ".name",
                 data::Tree::make(std::string(name)));
  return;
}

gboolean PJPresence::name_buddy_wrapped(gchar *name,
                                        gchar *buddy_uri,
                                        void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  context->sip_instance_->
      run_command_sync(std::bind(&PJPresence::name_buddy,
                                 context,
                                 std::string(name),
                                 std::string(buddy_uri)));
  return TRUE;
}

void
PJPresence::on_registration_state(pjsua_acc_id acc_id,
                                  pjsua_reg_info *info) {
  PJPresence *context =
      static_cast<PJPresence *>(pjsua_acc_get_user_data(acc_id));
  std::unique_lock<std::mutex> lock(context->registration_mutex_);
  if (PJ_SUCCESS != info->cbparam->status) {
    if (-1 != context->account_id_) {
      pj_status_t status = pjsua_acc_del(context->account_id_);
      if (PJ_SUCCESS != status)
        g_warning("Error deleting account after registration failled");
      context->account_id_ = -1;
    }
    context->registered_ = false;
    g_warning("registration failled");
  } else {
    context->registered_ = true;
  }
  g_debug("registration SIP status code %d\n", info->cbparam->code);
  context->registration_cond_.notify_one();
}

void PJPresence::on_buddy_state(pjsua_buddy_id buddy_id) {
  PJPresence *context =
      static_cast<PJPresence *>(pjsua_buddy_get_user_data(buddy_id));
  if (nullptr == context)
    return;
  pjsua_buddy_info info;
  pjsua_buddy_get_info(buddy_id, &info);

  std::string activity;
  if (PJRPID_ACTIVITY_UNKNOWN == info.rpid.activity)
    activity = "unknown";
  if (PJRPID_ACTIVITY_AWAY == info.rpid.activity)
    activity = "away";
  if (PJRPID_ACTIVITY_BUSY == info.rpid.activity)
    activity = "busy";

  g_debug("%.*s status is %.*s, subscription state is %s "
          "(last termination reason code=%d %.*s)\n"
          "rpid  activity %s, note %.*s\n",
          static_cast<int>(info.uri.slen),
          info.uri.ptr,
          static_cast<int>(info.status_text.slen),
          info.status_text.ptr,
          info.sub_state_name,
          info.sub_term_code,
          static_cast<int>(info.sub_term_reason.slen),
          info.sub_term_reason.ptr,
          activity.c_str(),
          static_cast<int>(info.rpid.note.slen),
          info.rpid.note.ptr);

  // std::string buddy_url(info.uri.ptr, (size_t) info.uri.slen);
  // tree->graft(".sip_url", data::Tree::make(buddy_url));
  std::string status("unknown");
  switch (info.status) {
    case PJSUA_BUDDY_STATUS_UNKNOWN:
      break;
    case PJSUA_BUDDY_STATUS_ONLINE:
      status = "online";
      break;
    case PJSUA_BUDDY_STATUS_OFFLINE:
      status = "offline";
      break;
    default:
      break;
  }
  if (PJRPID_ACTIVITY_AWAY == info.rpid.activity)
    status = "away";
  if (PJRPID_ACTIVITY_BUSY == info.rpid.activity)
    status = "busy";

  data::Tree::ptr tree = context->sip_instance_->
      prune_tree(std::string(".buddy." + std::to_string(buddy_id)),
                 false);  // do not signal since the tree will be updated
  if (!tree)
    tree = data::Tree::make();
  // writing status and state
  tree->graft(".status", data::Tree::make(status));
  tree->graft(".status_text",
              data::Tree::make(std::string(info.status_text.ptr,
                                          (size_t) info.status_text.slen)));
  tree->graft(".subscription_state",
              data::Tree::make(std::string(info.sub_state_name)));

  // replacing old one
  context->sip_instance_->
      graft_tree(std::string(".buddy." + std::to_string(buddy_id)), tree);
}

void PJPresence::set_status(const gint value, void *user_data) {
  printf("+++++++++++++++++++++ %s --begin\n", __FUNCTION__);
  PJPresence *context = static_cast<PJPresence *>(user_data);
  if (value < 0 || value >= OPT_MAX) {
    g_warning("invalid online status code");
    return;
  }
  if (-1 == context->account_id_) {
    g_warning("cannot set online status when not registered");
    return;
  }

  context->status_ = value;
  context->sip_instance_->
      run_command_sync(std::bind(&PJPresence::change_online_status,
                                 context,
                                 context->status_));
  GObjectWrapper::notify_property_changed(context->sip_instance_->gobject_->
                                          get_gobject(),
                                          context->status_enum_spec_);
  printf("+++++++++++++++++++++ %s --end\n", __FUNCTION__);
}

gint PJPresence::get_status(void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  return context->status_;
}

void PJPresence::change_online_status(gint status) {
  if (-1 == account_id_)
    return;
  pj_bool_t online_status = PJ_TRUE;
  pjrpid_element elem;
  pj_bzero(&elem, sizeof(elem));
  elem.type = PJRPID_ELEMENT_TYPE_PERSON;
  bool has_custom_status = true;
  char *tmp = nullptr;
  On_scope_exit {
    if (nullptr != tmp)
      g_free(tmp);
  };
  if (custom_status_.empty() || 0 == custom_status_.compare("")) {
    has_custom_status = false;
  } else {
    tmp = g_strdup(custom_status_.c_str());
    elem.note = pj_str(tmp);
  }

  switch (status) {
    case AVAILABLE:
      break;
    case BUSY:
      elem.activity = PJRPID_ACTIVITY_BUSY;
      if (!has_custom_status)
        pj_cstr(&elem.note, "Busy");
      break;
    case AWAY:
      elem.activity = PJRPID_ACTIVITY_AWAY;
      if (!has_custom_status)
        pj_cstr(&elem.note, "Away");
      break;
    case OFFLINE:
      online_status = PJ_FALSE;
      break;
  }

  pjsua_acc_set_online_status2(account_id_, online_status, &elem);
}

void PJPresence::set_note(const gchar *custom_status, void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  if (0 == context->custom_status_.compare(custom_status))
    return;
  context->custom_status_ = custom_status;
  context->sip_instance_->run_command_sync(std::bind
                                           (&PJPresence::change_online_status,
                                            context, context->status_));

  context->sip_instance_->custom_props_->
      notify_property_changed(context->custom_status_spec_);
}

const gchar *PJPresence::get_note(void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  return context->custom_status_.c_str();
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
void
PJPresence::on_incoming_subscribe(pjsua_acc_id acc_id,
                                  pjsua_srv_pres *srv_pres,
                                  pjsua_buddy_id buddy_id,
                                  const pj_str_t *from,
                                  pjsip_rx_data *rdata,
                                  pjsip_status_code *code,
                                  pj_str_t *reason,
                                  pjsua_msg_data *msg_data) {
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
void
PJPresence::on_buddy_evsub_state(pjsua_buddy_id buddy_id,
                                 pjsip_evsub *sub,
                                 pjsip_event *event) {
  char event_info[80];
  PJ_UNUSED_ARG(sub);
  event_info[0] = '\0';
  if (event->type == PJSIP_EVENT_TSX_STATE &&
      event->body.tsx_state.type == PJSIP_EVENT_RX_MSG) {
    pjsip_rx_data *rdata = event->body.tsx_state.src.rdata;
    snprintf(event_info, sizeof(event_info),
             " (RX %s)", pjsip_rx_data_get_info(rdata));
  }
  g_debug("Buddy %d: subscription state: %s (event: %s%s)",
         buddy_id, pjsip_evsub_get_state_name(sub),
         pjsip_event_str(event->type), event_info);
}

gboolean PJPresence::save_buddies_wrapped(gchar *file_name,
                                          void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);

  // HERE
  // auto serialize_buddies = [&] (data::Tree::ptrc tree) {
  //   data::Tree::ptr buds = tree->get("buddy");
  //   return data::BasicSerializer::serialize(bud);
  // };
  
  std::string buddies = context->sip_instance_->
      invoke_info_tree<std::string>(data::BasicSerializer::serialize); 
  return TRUE;
}

gboolean PJPresence::get_sip_registration_status(void *user_data) {
  PJPresence *context = static_cast<PJPresence *>(user_data);
  if (!context->registered_)
    return FALSE;
  return TRUE;
}
  
}  // namespace switcher
