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

#include <node.h>
#include <uv.h>
#include <iostream>
#include <utility>
#include "./switcher-controller.hpp"
#include "switcher/quiddity-manager.hpp"

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

using namespace std;
using namespace v8;
using namespace node;
using namespace switcher;

Persistent<Function> SwitcherController::constructor;

SwitcherController::SwitcherController(const std::string &name, Local<Function> logger_callback) :
    quiddity_manager(switcher::QuiddityManager::make_manager(name))
{
    Isolate* isolate = Isolate::GetCurrent();

    user_log_cb.Reset(isolate, logger_callback);

    // mutex
    uv_mutex_init(&switcher_log_mutex);
    uv_mutex_init(&switcher_prop_mutex);
    uv_mutex_init(&switcher_sig_mutex);

    // async
    switcher_log_async.data = this;
    uv_async_init(uv_default_loop(), &switcher_log_async, NotifyLog);

    switcher_prop_async.data = this;
    uv_async_init(uv_default_loop(), &switcher_prop_async, NotifyProp);

    switcher_sig_async.data = this;
    uv_async_init(uv_default_loop(), &switcher_sig_async, NotifySignal);

    // subscribers
    quiddity_manager->make_signal_subscriber("signal_sub", signal_cb, this);

    // init
    quiddity_manager->create("logger", "internal_logger");
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "shmdata", nullptr);
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "GStreamer", nullptr);
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "GLib", nullptr);
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "GLib-GObject", nullptr);
    quiddity_manager->use_prop<MPtr(&PContainer::set_str_str)>("internal_logger", "mute", "false");
    quiddity_manager->use_prop<MPtr(&PContainer::set_str_str)>("internal_logger", "debug", "true");
    quiddity_manager->use_prop<MPtr(&PContainer::set_str_str)>("internal_logger", "verbose", "true");
    auto last_line_id = quiddity_manager->use_prop<MPtr(&PContainer::get_id)>(
        "internal_logger", "last-line");
    auto manager_ptr = quiddity_manager.get();
    if (0 != last_line_id)
      quiddity_manager->use_prop<MPtr(&PContainer::subscribe)>(
          "internal_logger",
          last_line_id,
          [last_line_id, manager_ptr, this](){
            uv_mutex_lock(&switcher_log_mutex);
            switcher_log_list.push_back(
                manager_ptr->use_prop<MPtr(&PContainer::get<std::string>)>(
                    "internal_logger", last_line_id));
            uv_mutex_unlock(&switcher_log_mutex);
            uv_async_send(&switcher_log_async);
          },
          nullptr);

    quiddity_manager->create("create_remove_spy", "create_remove_spy");
    quiddity_manager->subscribe_signal("signal_sub", "create_remove_spy", "on-quiddity-created");
    quiddity_manager->subscribe_signal("signal_sub", "create_remove_spy", "on-quiddity-removed");

    // loading plugins
    // FIXME use config.h for having the appropriate version

#ifdef HAVE_CONFIG_H
    {
      std::string usr_plugin_dir("/usr/switcher-" LIBSWITCHER_API_VERSION "/plugins");
      quiddity_manager->scan_directory_for_plugins(usr_plugin_dir.c_str());
      std::string usr_local_plugin_dir("/usr/local/switcher-" LIBSWITCHER_API_VERSION "/plugins");
      quiddity_manager->scan_directory_for_plugins(usr_local_plugin_dir);
    }
#endif

    // do not play with previous config when saving
    quiddity_manager->reset_command_history(false);
};

SwitcherController::~SwitcherController() {};

void SwitcherController::Init(Handle<Object> exports) {
    Isolate* isolate = Isolate::GetCurrent();

    // Prepare constructor template
    Local<FunctionTemplate> tpl = FunctionTemplate::New(isolate, New);
    tpl->SetClassName(String::NewFromUtf8(isolate, "Switcher"));
    tpl->InstanceTemplate()->SetInternalFieldCount(35);

    // lifecycle - 1
    NODE_SET_PROTOTYPE_METHOD( tpl, "close", SwitcherClose );

    // history - 3
    NODE_SET_PROTOTYPE_METHOD( tpl, "save_history", SaveHistory );
    NODE_SET_PROTOTYPE_METHOD( tpl, "load_history_from_current_state", LoadHistoryFromCurrentState );
    NODE_SET_PROTOTYPE_METHOD( tpl, "load_history_from_scratch", LoadHistoryFromScratch );

    // life manager - 8
    NODE_SET_PROTOTYPE_METHOD( tpl, "create", Create );
    NODE_SET_PROTOTYPE_METHOD( tpl, "remove", Remove );
    NODE_SET_PROTOTYPE_METHOD( tpl, "has_quiddity", HasQuiddity );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_classes_doc", GetClassesDoc );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_class_doc", GetClassDoc );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_quiddity_description", GetQuiddityDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_quiddities_description", GetQuidditiesDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_info", GetInfo );

    // properties - 6
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_properties_description", GetPropertiesDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_property_description", GetPropertyDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_properties_description_by_class", GetPropertiesDescriptionByClass );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_property_description_by_class", GetPropertyDescriptionByClass );
    NODE_SET_PROTOTYPE_METHOD( tpl, "set_property_value", SetProperty );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_property_value", GetProperty );

    // methods - 5
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_methods_description", GetMethodsDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_method_description", GetMethodDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_methods_description_by_class", GetMethodsDescriptionByClass );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_method_description_by_class", GetMethodDescriptionByClass );
    NODE_SET_PROTOTYPE_METHOD( tpl, "invoke", Invoke );

    // property subscription - 4
    NODE_SET_PROTOTYPE_METHOD( tpl, "register_prop_callback", RegisterPropCallback );
    NODE_SET_PROTOTYPE_METHOD( tpl, "subscribe_to_property", SubscribeToProperty );
    NODE_SET_PROTOTYPE_METHOD( tpl, "unsubscribe_from_property", UnsubscribeFromProperty );

    // signals - 4
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_signals_description", GetSignalsDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_signal_description", GetSignalDescription );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_signals_description_by_class", GetSignalsDescriptionByClass );
    NODE_SET_PROTOTYPE_METHOD( tpl, "get_signal_description_by_class", GetSignalDescriptionByClass );

    // signal subscription - 4
    NODE_SET_PROTOTYPE_METHOD( tpl, "register_signal_callback", RegisterSignalCallback );
    NODE_SET_PROTOTYPE_METHOD( tpl, "subscribe_to_signal", SubscribeToSignal );
    NODE_SET_PROTOTYPE_METHOD( tpl, "unsubscribe_to_signal", UnsubscribeToSignal );
    NODE_SET_PROTOTYPE_METHOD( tpl, "list_subscribed_signals", ListSubscribedSignals );

    // constructor
    constructor.Reset(isolate, tpl->GetFunction());
    exports->Set(String::NewFromUtf8(isolate, "Switcher"), tpl->GetFunction());
}


void SwitcherController::release( ) {
  quiddity_manager.reset();

  if (!uv_is_closing((uv_handle_t*)&switcher_log_async)) {
    uv_close((uv_handle_t*)&switcher_log_async, nullptr);
  }
  if (!uv_is_closing((uv_handle_t*)&switcher_prop_async)) {
    uv_close((uv_handle_t*)&switcher_prop_async, nullptr);
  }
  if (!uv_is_closing((uv_handle_t*)&switcher_sig_async)) {
    uv_close((uv_handle_t*)&switcher_sig_async, nullptr);
  }

  uv_mutex_destroy(&switcher_sig_mutex);
  uv_mutex_destroy(&switcher_prop_mutex);
  uv_mutex_destroy(&switcher_log_mutex);

  user_log_cb.Reset();
  user_prop_cb.Reset();
  user_signal_cb.Reset();
}

void SwitcherController::New(const FunctionCallbackInfo<Value>& args) {
    Isolate* isolate = args.GetIsolate();
    HandleScope scope(isolate);

    if (args.IsConstructCall()) {
        // Invoked as constructor: `new SwitcherController(...)`
        if ( args.Length() < 2 || !args[0]->IsString() || !args[1]->IsFunction() ) {
            isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments. Switcher requires a name and logger callback.")));
            return;
        }

        // Construction
        String::Utf8Value name( ( args.Length() >= 1 && !args[0]->IsString() ) ? String::NewFromUtf8(isolate, "nodeserver") : args[0]->ToString());
        SwitcherController* obj = new SwitcherController(std::string(*name), Local<Function>::Cast(args[1]));
        obj->Wrap(args.This());
        args.GetReturnValue().Set(args.This());

      } else {
        // Invoked as plain function `MyObject(...)`, turn into construct call.
        const int argc = 1;
        Local<Value> argv[argc] = { args[0] };
        Local<Function> cons = Local<Function>::New(isolate, constructor);
        args.GetReturnValue().Set(cons->NewInstance(argc, argv));

      }
}

Handle<Value> SwitcherController::parseJson(Handle<Value> jsonString) {
    Isolate* isolate = Isolate::GetCurrent();
    HandleScope scope(isolate);

    Handle<Object> global = isolate->GetCurrentContext()->Global();

    Handle<Object> JSON = global->Get(String::NewFromUtf8(isolate, "JSON"))->ToObject();
    Handle<Function> JSON_parse = Handle<Function>::Cast(JSON->Get(String::NewFromUtf8(isolate, "parse")));

    return JSON_parse->Call(JSON, 1, &jsonString);
}

// void SwitcherController::logger_cb(const std::string& /*subscriber_name*/, const std::string& /*quiddity_name*/, const std::string& /*property_name*/, const std::string &value, void *user_data) {
//   SwitcherController *obj = static_cast<SwitcherController*>(user_data);
//   uv_mutex_lock(&obj->switcher_log_mutex);
//   obj->switcher_log_list.push_back(value);
//   uv_mutex_unlock(&obj->switcher_log_mutex);
//   uv_async_send(&obj->switcher_log_async);
// }

void SwitcherController::NotifyLog(uv_async_s* async) {
    Isolate* isolate = Isolate::GetCurrent();
    HandleScope scope(isolate);
    SwitcherController *obj = static_cast<SwitcherController*>(async->data);
    bool cleanup = false;

    if (!obj->user_log_cb.IsEmpty() ) {
        Local<Function> cb = Local<Function>::New(isolate, obj->user_log_cb);
        if ( cb->IsCallable() ) {
            TryCatch try_catch;
            uv_mutex_lock(&obj->switcher_log_mutex);
            for (auto &it: obj->switcher_log_list) {
                Local<Value> argv[] = { String::NewFromUtf8(isolate, it.c_str())};
                cb->Call(isolate->GetCurrentContext()->Global(), 1, argv);
            }
            obj->switcher_log_list.clear();
            uv_mutex_unlock(&obj->switcher_log_mutex);

            if (try_catch.HasCaught()) {
                FatalException(isolate, try_catch);
            }
        } else {
            cleanup = true;
        }
    } else {
        cleanup = true;
    }

    if ( cleanup ) {
            uv_mutex_lock(&obj->switcher_log_mutex);
            obj->switcher_log_list.clear();
            uv_mutex_unlock(&obj->switcher_log_mutex);
    }
}

// void SwitcherController::property_cb(const std::string& /*subscriber_name*/, const std::string &quiddity_name, const std::string &property_name, const std::string &value, void *user_data) {
//   SwitcherController *obj = static_cast<SwitcherController*>(user_data);
//   uv_mutex_lock(&obj->switcher_prop_mutex);
//   obj->switcher_prop_list.push_back(PropUpdate(std::move(quiddity_name), std::move(property_name), std::move(value)));
//   uv_mutex_unlock(&obj->switcher_prop_mutex);
//   uv_async_send(&obj->switcher_prop_async);
// }

void SwitcherController::NotifyProp(uv_async_s *async) {
    Isolate* isolate = Isolate::GetCurrent();
    HandleScope scope(isolate);
    SwitcherController *obj = static_cast<SwitcherController*>(async->data);

    bool cleanup = false;

    if (!obj->user_prop_cb.IsEmpty() ) {
        Local<Function> cb = Local<Function>::New(isolate, obj->user_prop_cb);
        if ( cb->IsCallable() ) {
            TryCatch try_catch;
            uv_mutex_lock(&obj->switcher_prop_mutex);
            for (auto &it: obj->switcher_prop_list) {
                Local<Value> argv[3];
                argv[0] = {String::NewFromUtf8(isolate, it.quid_.c_str())};
                argv[1] = {String::NewFromUtf8(isolate, it.prop_.c_str())};
                argv[2] = {String::NewFromUtf8(isolate, it.val_.c_str())};
                cb->Call(isolate->GetCurrentContext()->Global(), 3, argv);
            }
            obj->switcher_prop_list.clear();
            uv_mutex_unlock(&obj->switcher_prop_mutex);

            if (try_catch.HasCaught()) {
                FatalException(isolate, try_catch);
            }
        } else {
            cleanup = true;
        }
    } else {
        cleanup = true;
    }

    if ( cleanup ) {
        uv_mutex_lock(&obj->switcher_prop_mutex);
        obj->switcher_prop_list.clear();
        uv_mutex_unlock(&obj->switcher_prop_mutex);
    }
}

void SwitcherController::signal_cb(const std::string& /*subscriber_name*/, const std::string &quiddity_name, const std::string &signal_name, const std::vector<std::string> &params, void *user_data) {
  SwitcherController *obj = static_cast<SwitcherController*>(user_data);
  uv_mutex_lock(&obj->switcher_sig_mutex);
  obj->switcher_sig_list.push_back(SigUpdate(quiddity_name, signal_name, params));
  uv_mutex_unlock(&obj->switcher_sig_mutex);
  uv_async_send(&obj->switcher_sig_async);
}

void SwitcherController::NotifySignal(uv_async_s *async) {
    Isolate* isolate = Isolate::GetCurrent();
  HandleScope scope(isolate);
  SwitcherController *obj = static_cast<SwitcherController*>(async->data);

  bool cleanup = false;

  if (!obj->user_signal_cb.IsEmpty() ) {
      Local<Function> cb = Local<Function>::New(isolate, obj->user_signal_cb);

      if (!obj->user_signal_cb.IsEmpty() && cb->IsCallable()) {
        TryCatch try_catch;

        uv_mutex_lock(&obj->switcher_sig_mutex);
        // Performing a copy in order to avoid deadlock from signal handlers having to call the addon themselves
        // For example, on-quiddity-removed has to also remove associated quiddities
        auto sig_list = obj->switcher_sig_list;
        obj->switcher_sig_list.clear();
        uv_mutex_unlock(&obj->switcher_sig_mutex);

        for (auto &it: sig_list) {
          Local<Value> argv[3];
          Local<Array> array = Array::New(isolate, it.val_.size());
          for (auto &item: it.val_) {
            array->Set(0, String::NewFromUtf8(isolate, item.c_str()));
          }
          argv[0] = {String::NewFromUtf8(isolate, it.quid_.c_str())};
          argv[1] = {String::NewFromUtf8(isolate, it.sig_.c_str())};
          argv[2] = {array};

          cb->Call(isolate->GetCurrentContext()->Global(), 3, argv);
        }

        if (try_catch.HasCaught()) {
          FatalException(isolate, try_catch);
        }
    } else {
        cleanup = true;
    }
  } else {
    cleanup = true;
  }

  if ( cleanup ) {
      uv_mutex_lock(&obj->switcher_sig_mutex);
      obj->switcher_sig_list.clear();
      uv_mutex_unlock(&obj->switcher_sig_mutex);
  }
}

//  ██╗     ██╗███████╗███████╗ ██████╗██╗   ██╗ ██████╗██╗     ███████╗
//  ██║     ██║██╔════╝██╔════╝██╔════╝╚██╗ ██╔╝██╔════╝██║     ██╔════╝
//  ██║     ██║█████╗  █████╗  ██║      ╚████╔╝ ██║     ██║     █████╗
//  ██║     ██║██╔══╝  ██╔══╝  ██║       ╚██╔╝  ██║     ██║     ██╔══╝
//  ███████╗██║██║     ███████╗╚██████╗   ██║   ╚██████╗███████╗███████╗
//  ╚══════╝╚═╝╚═╝     ╚══════╝ ╚═════╝   ╚═╝    ╚═════╝╚══════╝╚══════╝

void SwitcherController::SwitcherClose(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());
  obj->release();
  args.GetReturnValue().Set(Boolean::New( isolate, true));
}

//  ██╗  ██╗██╗███████╗████████╗ ██████╗ ██████╗ ██╗   ██╗
//  ██║  ██║██║██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗╚██╗ ██╔╝
//  ███████║██║███████╗   ██║   ██║   ██║██████╔╝ ╚████╔╝
//  ██╔══██║██║╚════██║   ██║   ██║   ██║██╔══██╗  ╚██╔╝
//  ██║  ██║██║███████║   ██║   ╚██████╔╝██║  ██║   ██║
//  ╚═╝  ╚═╝╚═╝╚══════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝   ╚═╝

void SwitcherController::SaveHistory(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value file_path(args[0]->ToString());

  if (obj->quiddity_manager->save_command_history(std::string(*file_path).c_str())) {
    args.GetReturnValue().Set(Boolean::New( isolate, true));
  }
  args.GetReturnValue().Set(Boolean::New( isolate, false));
}

void SwitcherController::LoadHistoryFromCurrentState(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value file_path(args[0]->ToString());

  switcher::QuiddityManager::CommandHistory histo = obj->quiddity_manager->get_command_history_from_file(std::string(*file_path).c_str());

  if (histo.empty()) {
    args.GetReturnValue().Set(Boolean::New( isolate, false));
  }

  obj->quiddity_manager->play_command_history(histo, nullptr, nullptr, true);

  args.GetReturnValue().Set(Boolean::New( isolate, true));
}

void SwitcherController::LoadHistoryFromScratch(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value file_path(args[0]->ToString());

  switcher::QuiddityManager::CommandHistory histo = obj->quiddity_manager->get_command_history_from_file(std::string(*file_path).c_str());

  if (histo.empty()) {
    args.GetReturnValue().Set(Boolean::New( isolate, false));
  }

  obj->quiddity_manager->reset_command_history(true);
  obj->quiddity_manager->play_command_history(histo, nullptr, nullptr, true);

  args.GetReturnValue().Set(Boolean::New( isolate, true));
}

//   ██████╗ ██╗   ██╗██╗██████╗ ██████╗ ██╗████████╗██╗███████╗███████╗
//  ██╔═══██╗██║   ██║██║██╔══██╗██╔══██╗██║╚══██╔══╝██║██╔════╝██╔════╝
//  ██║   ██║██║   ██║██║██║  ██║██║  ██║██║   ██║   ██║█████╗  ███████╗
//  ██║▄▄ ██║██║   ██║██║██║  ██║██║  ██║██║   ██║   ██║██╔══╝  ╚════██║
//  ╚██████╔╝╚██████╔╝██║██████╔╝██████╔╝██║   ██║   ██║███████╗███████║
//   ╚══▀▀═╝  ╚═════╝ ╚═╝╚═════╝ ╚═════╝ ╚═╝   ╚═╝   ╚═╝╚══════╝╚══════╝

void SwitcherController::Remove(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "switcher remove: Wrong first arguments type")));
    return;
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (obj->quiddity_manager->remove(std::string(*first_arg))) {
    args.GetReturnValue().Set(Boolean::New( isolate, true));
  }
  args.GetReturnValue().Set(Boolean::New( isolate, false));
}

void SwitcherController::HasQuiddity(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "switcher has_quiddity: Wrong first arguments type")));
    return;
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (obj->quiddity_manager->has_quiddity(std::string(*first_arg))) {
    args.GetReturnValue().Set(Boolean::New( isolate, true));
  }
  args.GetReturnValue().Set(Boolean::New( isolate, false));
}

void SwitcherController::Create(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1 && args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "switcher create: Wrong first arg type")));
    return;
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (args.Length() == 2) {
    if (!args[1]->IsString()) {
      isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "switcher create: Wrong second arg type")));
      return;
    }

    String::Utf8Value second_arg(args[1]->ToString());

    args.GetReturnValue().Set(String::NewFromUtf8(isolate, obj->quiddity_manager->create(std::string(*first_arg), std::string(*second_arg)).c_str()));
  } else {
    args.GetReturnValue().Set(String::NewFromUtf8(isolate, obj->quiddity_manager->create(std::string(*first_arg)).c_str()));
  }
}

void SwitcherController::GetInfo(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "switcher get_info: Wrong first arg type")));
    return;
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (!args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "switcher get_info: Wrong second arg type")));
    return;
  }

  String::Utf8Value second_arg(args[1]->ToString());
  Handle<String> res =
      String::NewFromUtf8(isolate, obj->quiddity_manager->use_tree<MPtr(&InfoTree::serialize_json)>(std::string(*first_arg), std::string(*second_arg)).c_str());

  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetClassesDoc(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_classes_doc().c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetClassDoc(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_class_doc(std::string(*class_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetQuiddityDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value quiddity_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_quiddity_description(std::string(*quiddity_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetQuidditiesDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());
  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_quiddities_description().c_str());
  args.GetReturnValue().Set(parseJson(res));
}

//  ██████╗ ██████╗  ██████╗ ██████╗ ███████╗██████╗ ████████╗██╗███████╗███████╗
//  ██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔════╝██╔══██╗╚══██╔══╝██║██╔════╝██╔════╝
//  ██████╔╝██████╔╝██║   ██║██████╔╝█████╗  ██████╔╝   ██║   ██║█████╗  ███████╗
//  ██╔═══╝ ██╔══██╗██║   ██║██╔═══╝ ██╔══╝  ██╔══██╗   ██║   ██║██╔══╝  ╚════██║
//  ██║     ██║  ██║╚██████╔╝██║     ███████╗██║  ██║   ██║   ██║███████╗███████║
//  ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝╚══════╝╚══════╝

void SwitcherController::SetProperty(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 3) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());
  String::Utf8Value property_val(args[2]->ToString());

  Handle<Boolean> res =
      Boolean::New( isolate, obj->quiddity_manager->use_prop<MPtr(&PContainer::set_str_str)>(
          std::string(*element_name),
          std::string(*property_name),
          std::string(*property_val)));

  args.GetReturnValue().Set(res);
}

void SwitcherController::GetProperty(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<String> res =
      String::NewFromUtf8(isolate, obj->quiddity_manager->use_prop<MPtr(&PContainer::get_str_str)>(
          std::string(*element_name), std::string(*property_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetPropertiesDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_properties_description(std::string(*element_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetPropertyDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_property_description(std::string(*element_name), std::string(*property_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetPropertiesDescriptionByClass(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_properties_description_by_class(std::string(*class_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetPropertyDescriptionByClass(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_property_description_by_class(std::string(*class_name), std::string(*property_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

//  ███╗   ███╗███████╗████████╗██╗  ██╗ ██████╗ ██████╗ ███████╗
//  ████╗ ████║██╔════╝╚══██╔══╝██║  ██║██╔═══██╗██╔══██╗██╔════╝
//  ██╔████╔██║█████╗     ██║   ███████║██║   ██║██║  ██║███████╗
//  ██║╚██╔╝██║██╔══╝     ██║   ██╔══██║██║   ██║██║  ██║╚════██║
//  ██║ ╚═╝ ██║███████╗   ██║   ██║  ██║╚██████╔╝██████╔╝███████║
//  ╚═╝     ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝

void SwitcherController::Invoke(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() < 3) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsArray()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value method_name(args[1]->ToString());
  Local<Object> obj_arguments = args[2]->ToObject();
  Local<Array> arguments = obj_arguments->GetPropertyNames();

  std::vector<std::string> vector_arg;
  for (unsigned int i = 0; i < arguments->Length(); i++) {
    String::Utf8Value val(obj_arguments->Get(i)->ToString());
    vector_arg.push_back(std::string(*val));
  }

  std::string *return_value = nullptr;
  obj->quiddity_manager->invoke(std::string(*element_name), std::string(*method_name), &return_value, vector_arg);
  if (nullptr != return_value) {
    Handle<String> res = String::NewFromUtf8(isolate, (*return_value).c_str());
    delete return_value;  // FIXME this should not be necessary
    args.GetReturnValue().Set(parseJson(res));
  }

  return;
}

void SwitcherController::GetMethodsDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_methods_description(std::string(*element_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetMethodDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value method_name(args[1]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_method_description(std::string(*element_name), std::string(*method_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetMethodsDescriptionByClass(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_methods_description_by_class(std::string(*class_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetMethodDescriptionByClass(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());
  String::Utf8Value method_name(args[1]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_method_description_by_class(std::string(*class_name), std::string(*method_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

//  ██████╗ ██████╗  ██████╗ ██████╗ ███████╗██████╗ ████████╗██╗   ██╗
//  ██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔════╝██╔══██╗╚══██╔══╝╚██╗ ██╔╝
//  ██████╔╝██████╔╝██║   ██║██████╔╝█████╗  ██████╔╝   ██║    ╚████╔╝
//  ██╔═══╝ ██╔══██╗██║   ██║██╔═══╝ ██╔══╝  ██╔══██╗   ██║     ╚██╔╝
//  ██║     ██║  ██║╚██████╔╝██║     ███████╗██║  ██║   ██║      ██║
//  ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═╝   ╚═╝      ╚═╝

void SwitcherController::RegisterPropCallback(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  obj->user_prop_cb.Reset(isolate, Local<Function>::Cast(args[0]));

  return;
}

void SwitcherController::SubscribeToProperty(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  auto man = obj->quiddity_manager.get();
  auto qname = std::string(*element_name);
  auto pname = std::string(*property_name);
  auto prop_id = man->
      use_prop<MPtr(&PContainer::get_id)>(qname, pname);
  auto reg_id = man->use_prop<MPtr(&PContainer::subscribe)>(
          qname,
          prop_id,
          [obj, man, qname, pname, prop_id](){
            uv_mutex_lock(&obj->switcher_prop_mutex);
            obj->switcher_prop_list.push_back(PropUpdate(
                qname,
                pname,
                man->use_prop<MPtr(&PContainer::get_str)>(qname, prop_id)));
            uv_mutex_unlock(&obj->switcher_prop_mutex);
            uv_async_send(&obj->switcher_prop_async);
          },
          nullptr);
  if (0 == reg_id) {
    Handle<Boolean> res = Boolean::New( isolate, false);
    args.GetReturnValue().Set(res);
  }
  obj->prop_regs_[std::make_pair(qname,pname)] = reg_id;
  Handle<Boolean> res = Boolean::New( isolate, true);
  args.GetReturnValue().Set(res);
}

void SwitcherController::UnsubscribeFromProperty(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  auto qname = std::string(*element_name);
  auto pname = std::string(*property_name);
  auto it = obj->prop_regs_.find(std::make_pair(qname, pname));
  if (obj->prop_regs_.end() == it){
    Handle<Boolean> res = Boolean::New( isolate, false);
    args.GetReturnValue().Set(res);
  }
  auto man = obj->quiddity_manager.get();
  Handle<Boolean> res =
      Boolean::New( isolate, man->use_prop<MPtr(&PContainer::unsubscribe)>(
          qname,
          man->use_prop<MPtr(&PContainer::get_id)>(qname, pname),
          it->second));
  args.GetReturnValue().Set(res);
}

//  ███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗
//  ██╔════╝██║██╔════╝ ████╗  ██║██╔══██╗██║
//  ███████╗██║██║  ███╗██╔██╗ ██║███████║██║
//  ╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║
//  ███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗
//  ╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝

void SwitcherController::RegisterSignalCallback(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  obj->user_signal_cb.Reset(isolate, Local<Function>::Cast(args[0]));

  return;
}

void SwitcherController::SubscribeToSignal(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<Boolean> res = Boolean::New( isolate, obj->quiddity_manager->subscribe_signal(std::string("signal_sub"), std::string(*element_name), std::string(*signal_name)));
  args.GetReturnValue().Set(res);
}

void SwitcherController::UnsubscribeToSignal(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<Boolean> res = Boolean::New( isolate, obj->quiddity_manager->unsubscribe_signal(std::string("signal_sub"), std::string(*element_name), std::string(*signal_name)));
  args.GetReturnValue().Set(res);
}

void SwitcherController::ListSubscribedSignals(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->list_subscribed_signals_json("signal_sub").c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetSignalsDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_signals_description(std::string(*element_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetSignalDescription(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_signal_description(std::string(*element_name), std::string(*signal_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetSignalsDescriptionByClass(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_signals_description_by_class(std::string(*class_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}

void SwitcherController::GetSignalDescriptionByClass(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
  HandleScope scope(isolate);
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    isolate->ThrowException(Exception::TypeError(String::NewFromUtf8(isolate, "Wrong arguments")));
    return;
  }

  String::Utf8Value class_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<String> res = String::NewFromUtf8(isolate, obj->quiddity_manager->get_signal_description_by_class(std::string(*class_name), std::string(*signal_name)).c_str());
  args.GetReturnValue().Set(parseJson(res));
}
