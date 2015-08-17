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
#include "switcher/quiddity-manager.hpp"

class SwitcherController : public node::ObjectWrap {
    public:
        static void Init(v8::Handle<v8::Object> target);

        v8::Persistent<v8::Function> user_log_cb;                  // must be disposed
        v8::Persistent<v8::Function> user_prop_cb;                 // must be disposed
        v8::Persistent<v8::Function> user_signal_cb;               // must be disposed

    private:
        SwitcherController(const std::string &name, v8::Local<v8::Function> logger_callbac);
        ~SwitcherController();

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
        static void NotifySignal(uv_async_t *async, int status);

        static void property_cb(const std::string &subscriber_name, const std::string &quiddity_name, const std::string &property_name, const std::string &value, void *user_data);
        static void NotifyProp(uv_async_t *async, int status);

        static void logger_cb(const std::string &subscriber_name, const std::string &quiddity_name, const std::string &property_name, const std::string &value, void *user_data);
        static void NotifyLog(uv_async_t *async, int status);

        static v8::Handle<v8::Value> New(const v8::Arguments& args);
        static v8::Handle<v8::Value> SaveHistory(const v8::Arguments& args);
        static v8::Handle<v8::Value> LoadHistoryFromCurrentState(const v8::Arguments& args);
        static v8::Handle<v8::Value> LoadHistoryFromScratch(const v8::Arguments& args);
        static v8::Handle<v8::Value> Remove(const v8::Arguments& args);
        static v8::Handle<v8::Value> HasQuiddity(const v8::Arguments& args);
        static v8::Handle<v8::Value> Create(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetInfo(const v8::Arguments& args);
        static v8::Handle<v8::Value> SwitcherClose(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetClassesDoc(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetClassDoc(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetQuiddityDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetQuidditiesDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> SetProperty(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetProperty(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetPropertiesDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetPropertyDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetPropertiesDescriptionByClass(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetPropertyDescriptionByClass(const v8::Arguments& args);
        static v8::Handle<v8::Value> Invoke(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetMethodsDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetMethodDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetMethodsDescriptionByClass(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetMethodDescriptionByClass(const v8::Arguments& args);
        static v8::Handle<v8::Value> RegisterLogCallback(const v8::Arguments& args);
        static v8::Handle<v8::Value> RegisterPropCallback(const v8::Arguments& args);
        static v8::Handle<v8::Value> SubscribeToProperty(const v8::Arguments& args);
        static v8::Handle<v8::Value> UnsubscribeFromProperty(const v8::Arguments& args);
        static v8::Handle<v8::Value> ListSubscribedProperties(const v8::Arguments& args);
        static v8::Handle<v8::Value> RegisterSignalCallback(const v8::Arguments& args);
        static v8::Handle<v8::Value> SubscribeToSignal(const v8::Arguments& args);
        static v8::Handle<v8::Value> UnsubscribeToSignal(const v8::Arguments& args);
        static v8::Handle<v8::Value> ListSubscribedSignals(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetSignalsDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetSignalDescription(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetSignalsDescriptionByClass(const v8::Arguments& args);
        static v8::Handle<v8::Value> GetSignalDescriptionByClass(const v8::Arguments& args);
};

#endif
