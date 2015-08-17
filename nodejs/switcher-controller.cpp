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

#include <iostream>
#include <node.h>
#include "./switcher-controller.hpp"
#include "switcher/quiddity-manager.hpp"

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

using namespace std;
using namespace v8;
using namespace node;

SwitcherController::SwitcherController(const std::string &name, Local<Function> logger_callback) :
    quiddity_manager(switcher::QuiddityManager::make_manager(name))
{
    user_log_cb = Persistent<Function>::New(logger_callback);

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
    quiddity_manager->make_property_subscriber("log_sub", logger_cb, this);
    quiddity_manager->make_property_subscriber("prop_sub", property_cb, this);
    quiddity_manager->make_signal_subscriber("signal_sub", signal_cb, this);

    // init
    quiddity_manager->create("logger", "internal_logger");
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "shmdata", nullptr);
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "GStreamer", nullptr);
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "GLib", nullptr);
    quiddity_manager->invoke_va("internal_logger", "install_log_handler", nullptr, "GLib-GObject", nullptr);
    quiddity_manager->set_property("internal_logger", "mute", "false");
    quiddity_manager->set_property("internal_logger", "debug", "true");
    quiddity_manager->set_property("internal_logger", "verbose", "true");
    quiddity_manager->subscribe_property("log_sub", "internal_logger", "last-line");

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
    // Prepare constructor template
    Local<FunctionTemplate> tpl = FunctionTemplate::New(New);
    tpl->SetClassName(String::NewSymbol("Switcher"));
    tpl->InstanceTemplate()->SetInternalFieldCount(35);

    // lifecycle - 1
    tpl->PrototypeTemplate()->Set(String::NewSymbol("close"), FunctionTemplate::New(SwitcherClose)->GetFunction());

    // history - 3
    tpl->PrototypeTemplate()->Set(String::NewSymbol("save_history"), FunctionTemplate::New(SaveHistory)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("load_history_from_current_state"), FunctionTemplate::New(LoadHistoryFromCurrentState)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("load_history_from_scratch"), FunctionTemplate::New(LoadHistoryFromScratch)->GetFunction());

    // life manager - 8
    tpl->PrototypeTemplate()->Set(String::NewSymbol("create"), FunctionTemplate::New(Create)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("remove"), FunctionTemplate::New(Remove)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("has_quiddity"), FunctionTemplate::New(HasQuiddity)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_classes_doc"), FunctionTemplate::New(GetClassesDoc)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_class_doc"), FunctionTemplate::New(GetClassDoc)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_quiddity_description"), FunctionTemplate::New(GetQuiddityDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_quiddities_description"), FunctionTemplate::New(GetQuidditiesDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_info"), FunctionTemplate::New(GetInfo)->GetFunction());

    // properties - 6
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_properties_description"), FunctionTemplate::New(GetPropertiesDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_property_description"), FunctionTemplate::New(GetPropertyDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_properties_description_by_class"), FunctionTemplate::New(GetPropertiesDescriptionByClass)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_property_description_by_class"), FunctionTemplate::New(GetPropertyDescriptionByClass)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("set_property_value"), FunctionTemplate::New(SetProperty)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_property_value"), FunctionTemplate::New(GetProperty)->GetFunction());

    // methods - 5
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_methods_description"), FunctionTemplate::New(GetMethodsDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_method_description"), FunctionTemplate::New(GetMethodDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_methods_description_by_class"), FunctionTemplate::New(GetMethodsDescriptionByClass)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_method_description_by_class"), FunctionTemplate::New(GetMethodDescriptionByClass)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("invoke"), FunctionTemplate::New(Invoke)->GetFunction());

    // property subscription - 4
    tpl->PrototypeTemplate()->Set(String::NewSymbol("register_prop_callback"), FunctionTemplate::New(RegisterPropCallback)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("subscribe_to_property"), FunctionTemplate::New(SubscribeToProperty)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("unsubscribe_from_property"), FunctionTemplate::New(UnsubscribeFromProperty)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("list_subscribed_properties"), FunctionTemplate::New(ListSubscribedProperties)->GetFunction());

    // signals - 4
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_signals_description"), FunctionTemplate::New(GetSignalsDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_signal_description"), FunctionTemplate::New(GetSignalDescription)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_signals_description_by_class"), FunctionTemplate::New(GetSignalsDescriptionByClass)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("get_signal_description_by_class"), FunctionTemplate::New(GetSignalDescriptionByClass)->GetFunction());

    // signal subscription - 4
    tpl->PrototypeTemplate()->Set(String::NewSymbol("register_signal_callback"), FunctionTemplate::New(RegisterSignalCallback)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("subscribe_to_signal"), FunctionTemplate::New(SubscribeToSignal)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("unsubscribe_to_signal"), FunctionTemplate::New(UnsubscribeToSignal)->GetFunction());
    tpl->PrototypeTemplate()->Set(String::NewSymbol("list_subscribed_signals"), FunctionTemplate::New(ListSubscribedSignals)->GetFunction());

    // constructor
    Persistent<Function> constructor = Persistent<Function>::New(tpl->GetFunction());
    exports->Set(String::NewSymbol("Switcher"), constructor);
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

  user_log_cb.Dispose();
  user_prop_cb.Dispose();
  user_signal_cb.Dispose();
}

Handle<Value> SwitcherController::New(const Arguments& args) {
  HandleScope scope;

  if ( args.Length() < 2 || !args[0]->IsString() || !args[1]->IsFunction() ) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments. Switcher requires a name and logger callback.")));
    return scope.Close(Undefined());
  }

  // Construction
  String::Utf8Value name( ( args.Length() >= 1 && !args[0]->IsString() ) ? String::New("nodeserver") : args[0]->ToString());
  SwitcherController* obj = new SwitcherController(std::string(*name), Local<Function>::Cast(args[1]));

  obj->Wrap(args.This());

  return args.This();
}

Handle<Value> SwitcherController::parseJson(Handle<Value> jsonString) {
    HandleScope scope;

    Handle<Context> context = Context::GetCurrent();
    Handle<Object> global = context->Global();

    Handle<Object> JSON = global->Get(String::New("JSON"))->ToObject();
    Handle<Function> JSON_parse = Handle<Function>::Cast(JSON->Get(String::New("parse")));

    return scope.Close(JSON_parse->Call(JSON, 1, &jsonString));
}

void SwitcherController::logger_cb(const std::string& /*subscriber_name*/, const std::string& /*quiddity_name*/, const std::string& /*property_name*/, const std::string &value, void *user_data) {
  SwitcherController *obj = static_cast<SwitcherController*>(user_data);
  uv_mutex_lock(&obj->switcher_log_mutex);
  obj->switcher_log_list.push_back(value);
  uv_mutex_unlock(&obj->switcher_log_mutex);
  uv_async_send(&obj->switcher_log_async);
}

void SwitcherController::NotifyLog(uv_async_t* async, int /*status*/) {
  HandleScope scope;

  SwitcherController *obj = static_cast<SwitcherController*>(async->data);

  if (!obj->user_log_cb.IsEmpty() && obj->user_log_cb->IsCallable()) {
    TryCatch try_catch;
    uv_mutex_lock(&obj->switcher_log_mutex);
    for (auto &it: obj->switcher_log_list) {
      Local<Value> argv[] = { Local<Value>::New(String::New(it.c_str()))};
      obj->user_log_cb->Call(obj->user_log_cb, 1, argv);
    }
    obj->switcher_log_list.clear();
    uv_mutex_unlock(&obj->switcher_log_mutex);

    if (try_catch.HasCaught()) {
      node::FatalException(try_catch);
    }
  } else {
    uv_mutex_lock(&obj->switcher_log_mutex);
    obj->switcher_log_list.clear();
    uv_mutex_unlock(&obj->switcher_log_mutex);
  }
}

void SwitcherController::property_cb(const std::string& /*subscriber_name*/, const std::string &quiddity_name, const std::string &property_name, const std::string &value, void *user_data) {
  SwitcherController *obj = static_cast<SwitcherController*>(user_data);
  uv_mutex_lock(&obj->switcher_prop_mutex);
  obj->switcher_prop_list.push_back(PropUpdate(std::move(quiddity_name), std::move(property_name), std::move(value)));
  uv_mutex_unlock(&obj->switcher_prop_mutex);
  uv_async_send(&obj->switcher_prop_async);
}

void SwitcherController::NotifyProp(uv_async_t *async, int /*status*/) {
  HandleScope scope;
  SwitcherController *obj = static_cast<SwitcherController*>(async->data);

  if (!obj->user_prop_cb.IsEmpty() && obj->user_prop_cb->IsCallable()) {
    TryCatch try_catch;
    uv_mutex_lock(&obj->switcher_prop_mutex);
    for (auto &it: obj->switcher_prop_list) {
      Local<Value> argv[3];
      argv[0] = {Local<Value>::New(String::New(it.quid_.c_str()))};
      argv[1] = {Local<Value>::New(String::New(it.prop_.c_str()))};
      argv[2] = {Local<Value>::New(String::New(it.val_.c_str()))};
      obj->user_prop_cb->Call(obj->user_prop_cb, 3, argv);
    }
    obj->switcher_prop_list.clear();
    uv_mutex_unlock(&obj->switcher_prop_mutex);

    if (try_catch.HasCaught()) {
      node::FatalException(try_catch);
    }
  } else {
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

void SwitcherController::NotifySignal(uv_async_t *async, int /*status*/) {
  HandleScope scope;
  SwitcherController *obj = static_cast<SwitcherController*>(async->data);

  if (!obj->user_signal_cb.IsEmpty() && obj->user_signal_cb->IsCallable()) {
    TryCatch try_catch;

    uv_mutex_lock(&obj->switcher_sig_mutex);
    // Performing a copy in order to avoid deadlock from signal handlers having to call the addon themselves
    // For example, on-quiddity-removed has to also remove associated quiddities
    auto sig_list = obj->switcher_sig_list;
    obj->switcher_sig_list.clear();
    uv_mutex_unlock(&obj->switcher_sig_mutex);

    for (auto &it: sig_list) {
      Local<Value> argv[3];
      Local<Array> array = Array::New(it.val_.size());
      for (auto &item: it.val_) {
        array->Set(0, String::New(item.c_str()));
      }
      argv[0] = {Local<Value>::New(String::New(it.quid_.c_str()))};
      argv[1] = {Local<Value>::New(String::New(it.sig_.c_str()))};
      argv[2] = {Local<Value>::New(array)};
      obj->user_signal_cb->Call(obj->user_signal_cb, 3, argv);
    }

    if (try_catch.HasCaught()) {
      node::FatalException(try_catch);
    }
  } else {
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

Handle<Value> SwitcherController::SwitcherClose(const Arguments &args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());
  obj->release();
  return scope.Close(Boolean::New(true));
}

//  ██╗  ██╗██╗███████╗████████╗ ██████╗ ██████╗ ██╗   ██╗
//  ██║  ██║██║██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗╚██╗ ██╔╝
//  ███████║██║███████╗   ██║   ██║   ██║██████╔╝ ╚████╔╝
//  ██╔══██║██║╚════██║   ██║   ██║   ██║██╔══██╗  ╚██╔╝
//  ██║  ██║██║███████║   ██║   ╚██████╔╝██║  ██║   ██║
//  ╚═╝  ╚═╝╚═╝╚══════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝   ╚═╝

Handle<Value> SwitcherController::SaveHistory(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value file_path(args[0]->ToString());

  if (obj->quiddity_manager->save_command_history(std::string(*file_path).c_str())) {
    return scope.Close(Boolean::New(true));
  }
  return scope.Close(Boolean::New(false));
}

Handle<Value> SwitcherController::LoadHistoryFromCurrentState(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value file_path(args[0]->ToString());

  switcher::QuiddityManager::CommandHistory histo = obj->quiddity_manager->get_command_history_from_file(std::string(*file_path).c_str());

  if (histo.empty()) {
    return scope.Close(Boolean::New(false));
  }

  obj->quiddity_manager->play_command_history(histo, nullptr, nullptr, true);

  return scope.Close(Boolean::New(true));
}

Handle<Value> SwitcherController::LoadHistoryFromScratch(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value file_path(args[0]->ToString());

  switcher::QuiddityManager::CommandHistory histo = obj->quiddity_manager->get_command_history_from_file(std::string(*file_path).c_str());

  if (histo.empty()) {
    return scope.Close(Boolean::New(false));
  }

  obj->quiddity_manager->reset_command_history(true);
  obj->quiddity_manager->play_command_history(histo, nullptr, nullptr, true);

  return scope.Close(Boolean::New(true));
}

//   ██████╗ ██╗   ██╗██╗██████╗ ██████╗ ██╗████████╗██╗███████╗███████╗
//  ██╔═══██╗██║   ██║██║██╔══██╗██╔══██╗██║╚══██╔══╝██║██╔════╝██╔════╝
//  ██║   ██║██║   ██║██║██║  ██║██║  ██║██║   ██║   ██║█████╗  ███████╗
//  ██║▄▄ ██║██║   ██║██║██║  ██║██║  ██║██║   ██║   ██║██╔══╝  ╚════██║
//  ╚██████╔╝╚██████╔╝██║██████╔╝██████╔╝██║   ██║   ██║███████╗███████║
//   ╚══▀▀═╝  ╚═════╝ ╚═╝╚═════╝ ╚═════╝ ╚═╝   ╚═╝   ╚═╝╚══════╝╚══════╝

Handle<Value> SwitcherController::Remove(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("switcher remove: Wrong first arguments type")));
    return scope.Close(Undefined());
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (obj->quiddity_manager->remove(std::string(*first_arg))) {
    return scope.Close(Boolean::New(true));
  }
  return scope.Close(Boolean::New(false));
}

Handle<Value> SwitcherController::HasQuiddity(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("switcher has_quiddity: Wrong first arguments type")));
    return scope.Close(Undefined());
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (obj->quiddity_manager->has_quiddity(std::string(*first_arg))) {
    return scope.Close(Boolean::New(true));
  }
  return scope.Close(Boolean::New(false));
}

Handle<Value> SwitcherController::Create(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1 && args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("switcher create: Wrong first arg type")));
    return scope.Close(Undefined());
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (args.Length() == 2) {
    if (!args[1]->IsString()) {
      ThrowException(Exception::TypeError(String::New("switcher create: Wrong second arg type")));
      return scope.Close(Undefined());
    }

    String::Utf8Value second_arg(args[1]->ToString());

    return scope.Close(String::New(obj->quiddity_manager->create(std::string(*first_arg), std::string(*second_arg)).c_str()));
  } else {
    return scope.Close(String::New(obj->quiddity_manager->create(std::string(*first_arg)).c_str()));
  }
}

Handle<Value> SwitcherController::GetInfo(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("switcher get_info: Wrong first arg type")));
    return scope.Close(Undefined());
  }

  String::Utf8Value first_arg(args[0]->ToString());

  if (!args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("switcher get_info: Wrong second arg type")));
    return scope.Close(Undefined());
  }

  String::Utf8Value second_arg(args[1]->ToString());
  Handle<String> res = String::New(obj->quiddity_manager->get_info(std::string(*first_arg), std::string(*second_arg)).c_str());

  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetClassesDoc(const Arguments &args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  Handle<String> res = String::New(obj->quiddity_manager->get_classes_doc().c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetClassDoc(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_class_doc(std::string(*class_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetQuiddityDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value quiddity_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_quiddity_description(std::string(*quiddity_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetQuidditiesDescription(const Arguments &args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());
  Handle<String> res = String::New(obj->quiddity_manager->get_quiddities_description().c_str());
  return scope.Close(parseJson(res));
}

//  ██████╗ ██████╗  ██████╗ ██████╗ ███████╗██████╗ ████████╗██╗███████╗███████╗
//  ██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔════╝██╔══██╗╚══██╔══╝██║██╔════╝██╔════╝
//  ██████╔╝██████╔╝██║   ██║██████╔╝█████╗  ██████╔╝   ██║   ██║█████╗  ███████╗
//  ██╔═══╝ ██╔══██╗██║   ██║██╔═══╝ ██╔══╝  ██╔══██╗   ██║   ██║██╔══╝  ╚════██║
//  ██║     ██║  ██║╚██████╔╝██║     ███████╗██║  ██║   ██║   ██║███████╗███████║
//  ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝╚══════╝╚══════╝

Handle<Value> SwitcherController::SetProperty(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 3) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());
  String::Utf8Value property_val(args[2]->ToString());

  Handle<Boolean> res = Boolean::New(obj->quiddity_manager->set_property(std::string(*element_name), std::string(*property_name), std::string(*property_val)));

  return scope.Close(res);
}

Handle<Value> SwitcherController::GetProperty(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_property(std::string(*element_name), std::string(*property_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetPropertiesDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_properties_description(std::string(*element_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetPropertyDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_property_description(std::string(*element_name), std::string(*property_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetPropertiesDescriptionByClass(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_properties_description_by_class(std::string(*class_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetPropertyDescriptionByClass(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_property_description_by_class(std::string(*class_name), std::string(*property_name)).c_str());
  return scope.Close(parseJson(res));
}

//  ███╗   ███╗███████╗████████╗██╗  ██╗ ██████╗ ██████╗ ███████╗
//  ████╗ ████║██╔════╝╚══██╔══╝██║  ██║██╔═══██╗██╔══██╗██╔════╝
//  ██╔████╔██║█████╗     ██║   ███████║██║   ██║██║  ██║███████╗
//  ██║╚██╔╝██║██╔══╝     ██║   ██╔══██║██║   ██║██║  ██║╚════██║
//  ██║ ╚═╝ ██║███████╗   ██║   ██║  ██║╚██████╔╝██████╔╝███████║
//  ╚═╝     ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝

Handle<Value> SwitcherController::Invoke(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() < 3) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsArray()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
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
    Handle<String> res = String::New((*return_value).c_str());
    delete return_value;  // FIXME this should not be necessary
    return scope.Close(parseJson(res));
  }

  return scope.Close(Undefined());
}

Handle<Value> SwitcherController::GetMethodsDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_methods_description(std::string(*element_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetMethodDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value method_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_method_description(std::string(*element_name), std::string(*method_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetMethodsDescriptionByClass(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_methods_description_by_class(std::string(*class_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetMethodDescriptionByClass(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());
  String::Utf8Value method_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_method_description_by_class(std::string(*class_name), std::string(*method_name)).c_str());
  return scope.Close(parseJson(res));
}

//  ██████╗ ██████╗  ██████╗ ██████╗ ███████╗██████╗ ████████╗██╗   ██╗
//  ██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔════╝██╔══██╗╚══██╔══╝╚██╗ ██╔╝
//  ██████╔╝██████╔╝██║   ██║██████╔╝█████╗  ██████╔╝   ██║    ╚████╔╝
//  ██╔═══╝ ██╔══██╗██║   ██║██╔═══╝ ██╔══╝  ██╔══██╗   ██║     ╚██╔╝
//  ██║     ██║  ██║╚██████╔╝██║     ███████╗██║  ██║   ██║      ██║
//  ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═╝   ╚═╝      ╚═╝

Handle<Value> SwitcherController::RegisterPropCallback(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  obj->user_prop_cb = Persistent<Function>::New(Local<Function>::Cast(args[0]));

  return scope.Close(Undefined());
}

Handle<Value> SwitcherController::SubscribeToProperty(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<Boolean> res = Boolean::New(obj->quiddity_manager->subscribe_property(std::string("prop_sub"), std::string(*element_name), std::string(*property_name)));
  return scope.Close(res);
}

Handle<Value> SwitcherController::UnsubscribeFromProperty(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value property_name(args[1]->ToString());

  Handle<Boolean> res = Boolean::New(obj->quiddity_manager->unsubscribe_property(std::string("prop_sub"), std::string(*element_name), std::string(*property_name)));
  return scope.Close(res);
}

Handle<Value> SwitcherController::ListSubscribedProperties(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  Handle<String> res = String::New(obj->quiddity_manager->list_subscribed_properties_json("prop_sub").c_str());
  return scope.Close(parseJson(res));
}

//  ███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗
//  ██╔════╝██║██╔════╝ ████╗  ██║██╔══██╗██║
//  ███████╗██║██║  ███╗██╔██╗ ██║███████║██║
//  ╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║
//  ███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗
//  ╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝

Handle<Value> SwitcherController::RegisterSignalCallback(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  obj->user_signal_cb = Persistent<Function>::New(Local<Function>::Cast(args[0]));

  return scope.Close(Undefined());
}

Handle<Value> SwitcherController::SubscribeToSignal(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<Boolean> res = Boolean::New(obj->quiddity_manager->subscribe_signal(std::string("signal_sub"), std::string(*element_name), std::string(*signal_name)));
  return scope.Close(res);
}

Handle<Value> SwitcherController::UnsubscribeToSignal(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<Boolean> res = Boolean::New(obj->quiddity_manager->unsubscribe_signal(std::string("signal_sub"), std::string(*element_name), std::string(*signal_name)));
  return scope.Close(res);
}

Handle<Value> SwitcherController::ListSubscribedSignals(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  Handle<String> res = String::New(obj->quiddity_manager->list_subscribed_signals_json("signal_sub").c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetSignalsDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_signals_description(std::string(*element_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetSignalDescription(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value element_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_signal_description(std::string(*element_name), std::string(*signal_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetSignalsDescriptionByClass(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 1) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_signals_description_by_class(std::string(*class_name)).c_str());
  return scope.Close(parseJson(res));
}

Handle<Value> SwitcherController::GetSignalDescriptionByClass(const Arguments& args) {
  HandleScope scope;
  SwitcherController* obj = ObjectWrap::Unwrap<SwitcherController>(args.This());

  if (args.Length() != 2) {
    ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
    return scope.Close(Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(Exception::TypeError(String::New("Wrong arguments")));
    return scope.Close(Undefined());
  }

  String::Utf8Value class_name(args[0]->ToString());
  String::Utf8Value signal_name(args[1]->ToString());

  Handle<String> res = String::New(obj->quiddity_manager->get_signal_description_by_class(std::string(*class_name), std::string(*signal_name)).c_str());
  return scope.Close(parseJson(res));
}
