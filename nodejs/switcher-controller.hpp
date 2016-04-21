/*
 * This file is part of switcher-nodejs.
 *
 * switcher is free software: you can redistribute it and/or modify
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

#ifndef SWITCHERCTRL_H
#define SWITCHERCTRL_H

#include <node.h>
#include <node_object_wrap.h>
#include <uv.h>
#include "switcher/quiddity-manager.hpp"

class SwitcherController : public node::ObjectWrap {
 public:
  static void Init(v8::Handle<v8::Object> exports);

  v8::Persistent<v8::Function> user_log_cb;                  // must be disposed
  v8::Persistent<v8::Function> user_prop_cb;                 // must be disposed
  v8::Persistent<v8::Function> user_signal_cb;               // must be disposed

 private:
  SwitcherController(const std::string &name, v8::Local<v8::Function> logger_callbac);
  ~SwitcherController();

  static v8::Persistent<v8::Function> constructor;

  void release();

  switcher::QuiddityManager::ptr quiddity_manager;


  //async log
  uv_async_t switcher_log_async;
  uv_mutex_t switcher_log_mutex;  // protecting the list
  std::list<std::string> switcher_log_list;  // lines waiting to be pushed to js

  //async property update
  using PropUpdate = struct PropUpdate_t {
    PropUpdate_t(std::string q, std::string p, std::string v):quid_(q), prop_(p), val_(v) {}
    std::string quid_{};
    std::string prop_{};
    std::string val_{};
  };
  uv_async_t switcher_prop_async;
  uv_mutex_t switcher_prop_mutex;  // protecting the list
  std::list<PropUpdate> switcher_prop_list;  // lines waiting to be pushed to js
  std::map<
    std::pair<std::string, std::string>,
    switcher::PContainer::register_id_t
    > prop_regs_{};

  //async signals
  using SigUpdate = struct SigUpdate_t {
    SigUpdate_t(std::string q, std::string p, std::vector<std::string> v):
        quid_(q), sig_(p), val_(v) {}
    std::string quid_{};
    std::string sig_{};
    std::vector<std::string> val_{};
  };
  uv_async_t switcher_sig_async;
  uv_mutex_t switcher_sig_mutex;  // protecting the list
  std::list<SigUpdate> switcher_sig_list;  // lines waiting to be pushed to js

  static v8::Handle<v8::Value> parseJson(v8::Handle<v8::Value> jsonString);

  static void signal_cb(const std::string &subscriber_name, const std::string &quiddity_name, const std::string &signal_name, const std::vector<std::string> &params, void *user_data);
  static void NotifySignal(uv_async_s *async);

  // static void property_cb(const std::string &subscriber_name, const std::string &quiddity_name, const std::string &property_name, const std::string &value, void *user_data);
  static void NotifyProp(uv_async_s *async);

  // static void logger_cb(const std::string &subscriber_name, const std::string &quiddity_name, const std::string &property_name, const std::string &value, void *user_data);
  static void NotifyLog(uv_async_s *async);

  static void New(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void SaveHistory(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void LoadHistoryFromCurrentState(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void LoadHistoryFromScratch(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void Remove(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void HasQuiddity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void Create(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetInfo(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void SwitcherClose(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetClassesDoc(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetClassDoc(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetQuiddityDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetQuidditiesDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void SetProperty(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetProperty(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetPropertiesDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetPropertyDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetPropertiesDescriptionByClass(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetPropertyDescriptionByClass(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void Invoke(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetMethodsDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetMethodDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetMethodsDescriptionByClass(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetMethodDescriptionByClass(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void RegisterLogCallback(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void RegisterPropCallback(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void SubscribeToProperty(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void UnsubscribeFromProperty(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void ListSubscribedProperties(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void RegisterSignalCallback(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void SubscribeToSignal(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void UnsubscribeToSignal(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void ListSubscribedSignals(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetSignalsDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetSignalDescription(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetSignalsDescriptionByClass(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void GetSignalDescriptionByClass(const v8::FunctionCallbackInfo<v8::Value>& args);
};

#endif
