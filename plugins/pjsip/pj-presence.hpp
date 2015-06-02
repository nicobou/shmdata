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

#ifndef __SWITCHER_PJ_PRESENCE_H__
#define __SWITCHER_PJ_PRESENCE_H__

#include <pjsua-lib/pjsua.h>
#include <glib.h>               // gboolean
#include <glib-object.h>        // GEnumValue

#include <mutex>
#include <condition_variable>
#include <string>
#include <map>

namespace switcher {
class PJSIP;
class PJCall;

class PJPresence {
  friend PJSIP;
  friend PJCall;  // for account local uri

 public:
  PJPresence() = delete;
  explicit PJPresence(PJSIP *sip_instance);
  ~PJPresence();
  PJPresence(const PJPresence &) = delete;
  PJPresence &operator=(const PJPresence &) = delete;

  enum {
    AVAILABLE, BUSY, AWAY, OFFLINE, OPT_MAX
  };

 private:
  PJSIP *sip_instance_{nullptr};
  pjsua_acc_id account_id_{-1};
  pjsua_acc_config cfg_;
  std::mutex registration_mutex_{};
  std::condition_variable registration_cond_{};
  // online status
  GParamSpec *status_enum_spec_{nullptr};
  static GEnumValue status_enum_[5];
  gint status_;
  GParamSpec *custom_status_spec_{nullptr};
  std::string custom_status_{};
  // sip registration status (read only)
  bool registered_{false};
  GParamSpec *sip_reg_status_spec_{nullptr};
  // account info
  std::string sip_local_user_{};
  pj_pool_t *acc_info_pool_{nullptr};
  std::map<std::string, pjsua_buddy_id> buddy_id_{};
  // registration
  static void on_registration_state(pjsua_acc_id acc_id,
                                    pjsua_reg_info *info);
  void register_account(const std::string &sip_user,
                        const std::string &sip_password);
  static gboolean register_account_wrapped(gchar *user,
                                           gchar *password,
                                           void *user_data);
  void unregister_account();
  static gboolean unregister_account_wrapped(gpointer /*unused */ ,
                                             void *user_data);

  // buddies
  void add_buddy(const std::string &sip_user);
  void del_buddy(const std::string &sip_user);
  static gboolean add_buddy_wrapped(gchar *buddy_uri,
                                    void *user_data);
  static gboolean del_buddy_wrapped(gchar *buddy_uri,
                                    void *user_data);
  static void on_buddy_state(pjsua_buddy_id buddy_id);
  // static gboolean save_buddies_wrapped(gchar *file_name,
  //                                      void *user_data);
  // online status
  static void set_status(const gint value, void *user_data);
  static gint get_status(void *user_data);
  static void set_note(const gchar *cutom_status, void *user_data);
  static const gchar *get_note(void *user_data);
  void change_online_status(gint status);
  // SIP registration prop
  static gboolean get_sip_registration_status(void *user_data);
  // name buddy method
  static gboolean name_buddy_wrapped(gchar *name,
                                     gchar *uri,
                                     void *user_data);
  void name_buddy(std::string name, std::string sip_user);
  // FIXME this should get id with checking the domain also
  pjsua_buddy_id get_id_from_buddy_name(const std::string &name);
  // pjsip functions
  static void on_reg_state(pjsua_acc_id acc_id);
  static void on_incoming_subscribe(pjsua_acc_id acc_id,
                                    pjsua_srv_pres *srv_pres,
                                    pjsua_buddy_id buddy_id,
                                    const pj_str_t *from,
                                    pjsip_rx_data *rdata,
                                    pjsip_status_code *code,
                                    pj_str_t *reason,
                                    pjsua_msg_data *msg_data);
  static void on_buddy_evsub_state(pjsua_buddy_id buddy_id,
                                   pjsip_evsub * sub, pjsip_event *event);
};

}  // namespace switcher
#endif
