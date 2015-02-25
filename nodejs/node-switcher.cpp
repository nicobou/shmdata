/*
 * This file is part of switcher.
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

#include <locale.h>
#include <node.h>
#include <v8.h>
#include <uv.h>
#include "switcher/quiddity-manager.hpp"

static std::vector<switcher::QuiddityManager::ptr> switcher_container;
static v8::Persistent<v8::Function> user_log_cb;                  // must be disposed
static v8::Persistent<v8::Function> user_prop_cb;                 // must be disposed
static v8::Persistent<v8::Function> user_signal_cb;               // must be disposed

//async log
static uv_async_t switcher_log_async;
static uv_mutex_t switcher_log_mutex;  // protecting the list
static std::list<std::string> switcher_log_list;  // lines waiting to be pushed to js 

//async props
using PropUpdate = struct PropUpdate_t {
  PropUpdate_t(std::string q, std::string p, std::string v):
      quid_(q), prop_(p), val_(v) {}
  std::string quid_{};
  std::string prop_{};
  std::string val_{};
}; 
static uv_async_t switcher_prop_async;
static uv_mutex_t switcher_prop_mutex;  // protecting the list
static std::list<PropUpdate> switcher_prop_list;  // lines waiting to be pushed to js 

//async sigs
using SigUpdate = struct SigUpdate_t {
  SigUpdate_t(std::string q, std::string p, std::vector<std::string> v):
      quid_(q), sig_(p), val_(v) {}
  std::string quid_{};
  std::string sig_{};
  std::vector<std::string> val_{};
}; 
static uv_async_t switcher_sig_async;
static uv_mutex_t switcher_sig_mutex;  // protecting the list
static std::list<SigUpdate> switcher_sig_list;  // lines waiting to be pushed to js 

static bool switcher_is_loading = false;

struct async_req_log {
  uv_work_t req;
  std::string msg;
};

struct async_req_prop {
  uv_work_t req;
  std::string quiddity_name;
  std::string property_name;
  std::string value;
};

struct async_req_signal {
  uv_work_t req;
  std::string quiddity_name;
  std::string signal_name;
  std::vector<std::string> params;
};

//------------ history
v8::Handle<v8::Value> SaveHistory(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value file_path(args[0]->ToString());

  if (switcher_container[0]->save_command_history
      (std::string(*file_path).c_str())) {
    v8::Handle<v8::String> res = v8::String::New("true");
    return scope.Close(res);
  }
  v8::Handle<v8::String> res = v8::String::New("false");
  return scope.Close(res);
}

v8::Handle<v8::Value>
LoadHistoryFromCurrentState(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value file_path(args[0]->ToString());

  switcher_is_loading = true;
  switcher::QuiddityManager::CommandHistory histo =
      switcher_container[0]->get_command_history_from_file(std::
                                                           string(*file_path).
                                                           c_str());

  if (histo.empty()) {
    v8::Handle<v8::String> res = v8::String::New("false");
    return scope.Close(res);
  }

  switcher_container[0]->play_command_history(histo, nullptr, nullptr, true);
  v8::Handle<v8::String> res = v8::String::New("true");

  switcher_is_loading = false;
  return scope.Close(res);
}

v8::Handle<v8::Value> LoadHistoryFromScratch(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value file_path(args[0]->ToString());

  switcher_is_loading = true;

  switcher::QuiddityManager::CommandHistory histo =
      switcher_container[0]->get_command_history_from_file(std::
                                                           string(*file_path).
                                                           c_str());

  if (histo.empty()) {
    v8::Handle<v8::String> res = v8::String::New("false");
    return scope.Close(res);
  }

  switcher_container[0]->reset_command_history(true);

  switcher_container[0]->play_command_history(histo, nullptr, nullptr, true);
  v8::Handle<v8::String> res = v8::String::New("true");

  switcher_is_loading = false;
  return scope.Close(res);
}

// ----------- life management
v8::Handle<v8::Value> Remove(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("switcher remove: Wrong first arguments type")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value first_arg(args[0]->ToString());
  if (switcher_container[0]->remove(std::string(*first_arg)))
    return v8::Boolean::New(true);
  else
    return v8::Boolean::New(false);
}

v8::Handle<v8::Value> Create(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1 && args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("switcher create: Wrong first arg type")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value first_arg(args[0]->ToString());
  v8::Local<v8::String> name;
  if (args.Length() == 2) {
    if (!args[1]->IsString()) {
      ThrowException(v8::Exception::TypeError(v8::String::New
                                              ("switcher create: Wrong second arg type")));
      return scope.Close(v8::Undefined());
    }
    v8::String::Utf8Value second_arg(args[1]->ToString());
    name =
        v8::String::New(switcher_container[0]->create
                        (std::string(*first_arg),
                         std::string(*second_arg)).c_str());
  }
  else
    name =
        v8::String::New(switcher_container[0]->create(std::string(*first_arg)).
                        c_str());

  return scope.Close(name);
}

v8::Handle<v8::Value> GetInfo(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("switcher get_info: Wrong first arg type")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value first_arg(args[0]->ToString());
  v8::Local<v8::String> name;
  if (!args[1]->IsString()) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("switcher get_info: Wrong second arg type")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value second_arg(args[1]->ToString());
  name =
      v8::String::New(switcher_container[0]->get_info
                      (std::string(*first_arg),
                       std::string(*second_arg)).c_str());
  return scope.Close(name);
}

v8::Handle<v8::Value> SwitcherClose(const v8::Arguments &args) {
  v8::HandleScope scope;
  // if (!user_log_cb.IsEmpty ())
  //  user_log_cb.Dispose ();
  // removing reference to manager in order to delete it
  uv_close((uv_handle_t*)&switcher_log_async, nullptr);
  uv_close((uv_handle_t*)&switcher_prop_async, nullptr);
  uv_close((uv_handle_t*)&switcher_sig_async, nullptr);
  switcher_container.clear();
  v8::Local<v8::String> name = v8::String::New("closed");
  return scope.Close(name);
}

v8::Handle<v8::Value> GetClassesDoc(const v8::Arguments &args) {
  v8::HandleScope scope;

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_classes_doc().c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetClassDoc(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::
      New(switcher_container[0]->get_class_doc(std::string(*class_name)).
          c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetQuiddityDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value quiddity_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_quiddity_description
                      (std::string(*quiddity_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetQuidditiesDescription(const v8::Arguments &args) {
  v8::HandleScope scope;

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->
                      get_quiddities_description().c_str());
  return scope.Close(res);
}

// end life manager

// ----------- properties
v8::Handle<v8::Value> SetProperty(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 3) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value property_name(args[1]->ToString());
  v8::String::Utf8Value property_val(args[2]->ToString());

  v8::Handle<v8::Boolean> res =
      v8::Boolean::New(switcher_container[0]->set_property
                       (std::string(*element_name),
                        std::string(*property_name),
                        std::string(*property_val)));
  return scope.Close(res);
}

v8::Handle<v8::Value> GetProperty(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value property_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_property
                      (std::string(*element_name),
                       std::string(*property_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetPropertiesDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_properties_description
                      (std::string(*element_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetPropertyDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value property_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_property_description
                      (std::string(*element_name),
                       std::string(*property_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value>
GetPropertiesDescriptionByClass(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::
      New(switcher_container[0]->get_properties_description_by_class
          (std::string(*class_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value>
GetPropertyDescriptionByClass(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());
  v8::String::Utf8Value property_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_property_description_by_class
                      (std::string(*class_name),
                       std::string(*property_name)).c_str());
  return scope.Close(res);
}

// end properties

// ----------- methods
v8::Handle<v8::Value> Invoke(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() < 3) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsArray()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value method_name(args[1]->ToString());
  v8::Local<v8::Object> obj_arguments = args[2]->ToObject();
  v8::Local<v8::Array> arguments = obj_arguments->GetPropertyNames();

  std::vector<std::string> vector_arg;
  for (unsigned int i = 0; i < arguments->Length(); i++) {
    v8::String::Utf8Value val(obj_arguments->Get(i)->ToString());
    vector_arg.push_back(std::string(*val));
  }

  std::string *return_value = nullptr;
  switcher_container[0]->invoke(std::string(*element_name),
                                std::string(*method_name),
                                &return_value, vector_arg);
  if (nullptr != return_value) {
    v8::Handle<v8::String> res = v8::String::New((*return_value).c_str());
    delete return_value;  // FIXME this should not be necessary
    return scope.Close(res);
  }
  v8::Handle<v8::String> res = v8::String::New("");
  return scope.Close(res);
}

v8::Handle<v8::Value> GetMethodsDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_methods_description
                      (std::string(*element_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetMethodDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value method_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_method_description
                      (std::string(*element_name),
                       std::string(*method_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value>
GetMethodsDescriptionByClass(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_methods_description_by_class
                      (std::string(*class_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value>
GetMethodDescriptionByClass(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());
  v8::String::Utf8Value method_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_method_description_by_class
                      (std::string(*class_name),
                       std::string(*method_name)).c_str());
  return scope.Close(res);
}

// log callback
v8::Handle<v8::Value> RegisterLogCallback(const v8::Arguments &args) {
  v8::HandleScope scope;
  user_log_cb =
      v8::Persistent<v8::Function>::New(v8::Local <
                                           v8::Function >::Cast(args[0]));
  // const unsigned argc = 1;
  // v8::Local<v8::Value> argv[argc] = { v8::Local<v8::Value>::New(v8::String::New("hello world")) };
  // user_log_cb->Call(v8::Context::GetCurrent()->Global(), argc, argv);

  return scope.Close(v8::Undefined());
}

void
NotifyLog(uv_async_t *async, int status) {
  v8::HandleScope scope;
  v8::TryCatch try_catch;
  uv_mutex_lock(&switcher_log_mutex);
  for (auto &it: switcher_log_list) {
    v8::Local<v8::Value> argv[] =
        { v8::Local<v8::Value>::New(v8::String::New(it.c_str()))};
    if (!user_log_cb.IsEmpty())
      if (user_log_cb->IsCallable())
        user_log_cb->Call(user_log_cb, 1, argv);
  }
  switcher_log_list.clear();
  uv_mutex_unlock(&switcher_log_mutex);
  if (try_catch.HasCaught()) {
    node::FatalException(try_catch);
  }
}

// call client log callback
static void
logger_cb(std::string subscriber_name,
          std::string quiddity_name,
          std::string property_name,
          std::string value,
          void *user_data) {
  uv_mutex_lock(&switcher_log_mutex);
  switcher_log_list.push_back(value);
  uv_mutex_unlock(&switcher_log_mutex);
  uv_async_send(&switcher_log_async);
}

// prop callback
v8::Handle<v8::Value> RegisterPropCallback(const v8::Arguments &args) {
  v8::HandleScope scope;
  user_prop_cb =
      v8::Persistent<v8::Function>::New(v8::Local<v8::Function>::Cast(args[0]));
  // const unsigned argc = 3;
  // v8::Local<v8::Value> argv[argc];
  // argv [0] = { v8::Local<v8::Value>::New(v8::String::New("hw1")) };
  // argv [1] = { v8::Local<v8::Value>::New(v8::String::New("hw2")) };
  // argv [2] = { v8::Local<v8::Value>::New(v8::String::New("hw3")) };
  // user_prop_cb->Call(v8::Context::GetCurrent()->Global(), argc, argv);
  return scope.Close(v8::Undefined());
}

void
NotifyProp(uv_async_t *async, int status) {
  v8::HandleScope scope;
  v8::TryCatch try_catch;
  uv_mutex_lock(&switcher_prop_mutex);
  for (auto &it: switcher_prop_list) {
    v8::Local<v8::Value> argv[3];
    argv[0] =
        {v8::Local<v8::Value>::New(v8::String::New(it.quid_.c_str()))};
    argv[1] =
        {v8::Local<v8::Value>::New(v8::String::New(it.prop_.c_str()))};
    argv[2] =
        {v8::Local<v8::Value>::New(v8::String::New(it.val_.c_str()))};
    if (!user_prop_cb.IsEmpty())
      if (user_prop_cb->IsCallable())
        user_prop_cb->Call(user_prop_cb, 3, argv);
  }
  switcher_prop_list.clear();
  uv_mutex_unlock(&switcher_prop_mutex);
  if (try_catch.HasCaught()) {
    node::FatalException(try_catch);
  }
}

// call client prop callback
static void
property_cb(std::string subscriber,
            std::string quid,
            std::string prop,
            std::string val,
            void *user_data) {
  uv_mutex_lock(&switcher_prop_mutex);
  switcher_prop_list.push_back(PropUpdate(std::move(quid),
                                          std::move(prop),
                                          std::move(val)));
  uv_mutex_unlock(&switcher_prop_mutex);
  uv_async_send(&switcher_prop_async);
}

v8::Handle<v8::Value> SubscribeToProperty(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(
        v8::String::New("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value property_name(args[1]->ToString());

  v8::Handle<v8::Boolean> res =
      v8::Boolean::New(switcher_container[0]->subscribe_property
                       (std::string("prop_sub"), std::string(*element_name),
                        std::string(*property_name)));
  return scope.Close(res);
}

v8::Handle<v8::Value> UnsubscribeToProperty(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value property_name(args[1]->ToString());

  v8::Handle<v8::Boolean> res =
      v8::Boolean::New(switcher_container[0]->unsubscribe_property
                       (std::string("prop_sub"), std::string(*element_name),
                        std::string(*property_name)));
  return scope.Close(res);
}

v8::Handle<v8::Value> ListSubscribedProperties(const v8::Arguments &args) {
  v8::HandleScope scope;

  v8::Handle<v8::String> res =
      v8::String::
      New(switcher_container[0]->list_subscribed_properties_json("prop_sub").
          c_str());
  return scope.Close(res);
}

// signal callback
v8::Handle<v8::Value> RegisterSignalCallback(const v8::Arguments &args) {
  v8::HandleScope scope;
  user_signal_cb =
      v8::Persistent<v8::Function>::New(v8::Local <
                                           v8::Function >::Cast(args[0]));
  // const unsigned argc = 3;
  // v8::Local<v8::Value> argv[argc];
  // argv [0] = { v8::Local<v8::Value>::New(v8::String::New("hw1")) };
  // argv [1] = { v8::Local<v8::Value>::New(v8::String::New("hw2")) };
  // argv [2] = { v8::Local<v8::Value>::New(v8::String::New("hw3")) };
  // user_prop_cb->Call(v8::Context::GetCurrent()->Global(), argc, argv);
  return scope.Close(v8::Undefined());
}

void
NotifySignal(uv_async_t */*async*/, int /*status*/) {
  v8::HandleScope scope;
  v8::TryCatch try_catch;
  uv_mutex_lock(&switcher_sig_mutex);
  // performing a copy in order to avoid
  // deadlock from unknown behavior in user_signal_cb
  auto sig_list = switcher_sig_list;
  switcher_sig_list.clear();
  uv_mutex_unlock(&switcher_sig_mutex);

  for (auto &it: sig_list) {
    v8::Local<v8::Value> argv[3];
    v8::Local<v8::Array> array = v8::Array::New(it.val_.size());
    for (auto &item: it.val_)
      array->Set(0, v8::String::New(item.c_str()));
    argv[0] = {v8::Local<v8::Value>::New(v8::String::New(it.quid_.c_str()))};
    argv[1] = {v8::Local<v8::Value>::New(v8::String::New(it.sig_.c_str()))};
    argv[2] = {v8::Local<v8::Value>::New(array)};
    if (!user_signal_cb.IsEmpty())
      if (user_signal_cb->IsCallable())
        user_signal_cb->Call(user_signal_cb, 3, argv);
  }
  if (try_catch.HasCaught()) {
    node::FatalException(try_catch);
  }
}

// call client signal callback
static void
signal_cb(std::string subscriber_name,
          std::string quid,
          std::string sig,
          std::vector<std::string> params,
          void *user_data) {
  uv_mutex_lock(&switcher_sig_mutex);
  switcher_sig_list.push_back(SigUpdate(quid,
                                        sig,
                                        params));
  uv_mutex_unlock(&switcher_sig_mutex);
  uv_async_send(&switcher_sig_async);
}

v8::Handle<v8::Value> SubscribeToSignal(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value signal_name(args[1]->ToString());

  v8::Handle<v8::Boolean> res =
      v8::Boolean::New(switcher_container[0]->subscribe_signal
                       (std::string("signal_sub"),
                        std::string(*element_name), std::string(*signal_name)));
  return scope.Close(res);
}

v8::Handle<v8::Value> UnsubscribeToSignal(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value signal_name(args[1]->ToString());

  v8::Handle<v8::Boolean> res =
      v8::Boolean::New(switcher_container[0]->unsubscribe_signal
                       (std::string("signal_sub"),
                        std::string(*element_name), std::string(*signal_name)));
  return scope.Close(res);
}

v8::Handle<v8::Value> ListSubscribedSignals(const v8::Arguments &args) {
  v8::HandleScope scope;

  v8::Handle<v8::String> res =
      v8::String::
      New(switcher_container[0]->list_subscribed_signals_json("signal_sub").
          c_str());
  return scope.Close(res);
}

// signal description
v8::Handle<v8::Value> GetSignalsDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_signals_description
                      (std::string(*element_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value> GetSignalDescription(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value element_name(args[0]->ToString());
  v8::String::Utf8Value signal_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_signal_description
                      (std::string(*element_name),
                       std::string(*signal_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value>
GetSignalsDescriptionByClass(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 1) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_signals_description_by_class
                      (std::string(*class_name)).c_str());
  return scope.Close(res);
}

v8::Handle<v8::Value>
GetSignalDescriptionByClass(const v8::Arguments &args) {
  v8::HandleScope scope;
  if (args.Length() != 2) {
    ThrowException(v8::Exception::TypeError(v8::String::New
                                            ("Wrong number of arguments")));
    return scope.Close(v8::Undefined());
  }
  if (!args[0]->IsString() || !args[1]->IsString()) {
    ThrowException(v8::
                   Exception::TypeError(v8::String::New("Wrong arguments")));
    return scope.Close(v8::Undefined());
  }
  v8::String::Utf8Value class_name(args[0]->ToString());
  v8::String::Utf8Value signal_name(args[1]->ToString());

  v8::Handle<v8::String> res =
      v8::String::New(switcher_container[0]->get_signal_description_by_class
                      (std::string(*class_name),
                       std::string(*signal_name)).c_str());
  return scope.Close(res);
}

// ------------ node init functions -------------------------------
void
Init(v8::Handle<v8::Object> target) {
  setlocale(LC_ALL, "");
  switcher::QuiddityManager::ptr switcher_manager
      = switcher::QuiddityManager::make_manager("nodeserver");

  switcher_container.push_back(switcher_manager); // keep reference only in the global container

  // loading plugins
  // FIXME use config.h for having the appropriate version
  gchar *usr_plugin_dir = g_strdup_printf("/usr/switcher-0.6/plugins");
  switcher_manager->scan_directory_for_plugins(usr_plugin_dir);
  g_free(usr_plugin_dir);
  
  gchar *usr_local_plugin_dir = g_strdup_printf("/usr/local/switcher-0.6/plugins");
  switcher_manager->scan_directory_for_plugins(usr_local_plugin_dir);
  g_free(usr_local_plugin_dir);


  // logs
  uv_mutex_init(&switcher_log_mutex);
  uv_async_init(uv_default_loop(), &switcher_log_async, NotifyLog);

  switcher_manager->create("logger", "internal_logger");
  switcher_manager->invoke_va("internal_logger", "install_log_handler",
                              nullptr, "shmdata", nullptr);
  switcher_manager->invoke_va("internal_logger", "install_log_handler",
                              nullptr, "GStreamer", nullptr);
  switcher_manager->invoke_va("internal_logger", "install_log_handler",
                              nullptr, "GLib", nullptr);
  switcher_manager->invoke_va("internal_logger", "install_log_handler",
                              nullptr, "GLib-GObject", nullptr);
  switcher_manager->set_property("internal_logger", "mute", "false");
  switcher_manager->set_property("internal_logger", "debug", "true");
  switcher_manager->set_property("internal_logger", "verbose", "true");
  switcher_manager->make_property_subscriber("log_sub", logger_cb, nullptr);
  switcher_manager->subscribe_property("log_sub", "internal_logger",
                                       "last-line");

  // props
  uv_mutex_init(&switcher_prop_mutex);
  uv_async_init(uv_default_loop(), &switcher_prop_async, NotifyProp);
  switcher_manager->make_property_subscriber("prop_sub", property_cb, nullptr);

  // signals
  uv_mutex_init(&switcher_sig_mutex);
  uv_async_init(uv_default_loop(), &switcher_sig_async, NotifySignal);
  switcher_manager->make_signal_subscriber("signal_sub", signal_cb, nullptr);
  switcher_manager->create("create_remove_spy", "create_remove_spy");
  switcher_manager->subscribe_signal("signal_sub", "create_remove_spy",
                                     "on-quiddity-created");
  switcher_manager->subscribe_signal("signal_sub", "create_remove_spy",
                                     "on-quiddity-removed");

  // do not play with previous config when saving
  switcher_manager->reset_command_history(false);

  // -------- registering functions ----------------
  // history
  target->Set(v8::String::NewSymbol("save_history"),
              v8::FunctionTemplate::New(SaveHistory)->GetFunction());
  target->Set(v8::String::NewSymbol("load_history_from_current_state"),
              v8::FunctionTemplate::
              New(LoadHistoryFromCurrentState)->GetFunction());
  target->Set(v8::String::NewSymbol("load_history_from_scratch"),
              v8::FunctionTemplate::
              New(LoadHistoryFromScratch)->GetFunction());

  // life manager
  target->Set(v8::String::NewSymbol("create"),
              v8::FunctionTemplate::New(Create)->GetFunction());
  target->Set(v8::String::NewSymbol("remove"),
              v8::FunctionTemplate::New(Remove)->GetFunction());
  target->Set(v8::String::NewSymbol("close"),
              v8::FunctionTemplate::New(SwitcherClose)->GetFunction());
  target->Set(v8::String::NewSymbol("get_classes_doc"),
              v8::FunctionTemplate::New(GetClassesDoc)->GetFunction());
  target->Set(v8::String::NewSymbol("get_class_doc"),
              v8::FunctionTemplate::New(GetClassDoc)->GetFunction());
  target->Set(v8::String::NewSymbol("get_quiddity_description"),
              v8::FunctionTemplate::
              New(GetQuiddityDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_quiddities_description"),
              v8::FunctionTemplate::
              New(GetQuidditiesDescription)->GetFunction());

  target->Set(v8::String::NewSymbol("get_info"),
              v8::FunctionTemplate::New(GetInfo)->GetFunction());

  // properties
  target->Set(v8::String::NewSymbol("get_properties_description"),
              v8::FunctionTemplate::
              New(GetPropertiesDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_property_description"),
              v8::FunctionTemplate::
              New(GetPropertyDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_properties_description_by_class"),
              v8::FunctionTemplate::
              New(GetPropertiesDescriptionByClass)->GetFunction());
  target->Set(v8::String::NewSymbol("get_property_description_by_class"),
              v8::FunctionTemplate::
              New(GetPropertyDescriptionByClass)->GetFunction());
  target->Set(v8::String::NewSymbol("set_property_value"),
              v8::FunctionTemplate::New(SetProperty)->GetFunction());
  target->Set(v8::String::NewSymbol("get_property_value"),
              v8::FunctionTemplate::New(GetProperty)->GetFunction());
  // methods
  target->Set(v8::String::NewSymbol("get_methods_description"),
              v8::FunctionTemplate::
              New(GetMethodsDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_method_description"),
              v8::FunctionTemplate::New(GetMethodDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_methods_description_by_class"),
              v8::FunctionTemplate::
              New(GetMethodsDescriptionByClass)->GetFunction());
  target->Set(v8::String::NewSymbol("get_method_description_by_class"),
              v8::FunctionTemplate::
              New(GetMethodDescriptionByClass)->GetFunction());
  target->Set(v8::String::NewSymbol("invoke"),
              v8::FunctionTemplate::New(Invoke)->GetFunction());

  // log
  target->Set(v8::String::NewSymbol("register_log_callback"),
              v8::FunctionTemplate::New(RegisterLogCallback)->GetFunction());

  // property subscription
  target->Set(v8::String::NewSymbol("register_prop_callback"),
              v8::FunctionTemplate::New(RegisterPropCallback)->GetFunction());
  target->Set(v8::String::NewSymbol("subscribe_to_property"),
              v8::FunctionTemplate::New(SubscribeToProperty)->GetFunction());
  target->Set(v8::String::NewSymbol("unsubscribe_to_property"),
              v8::FunctionTemplate::
              New(UnsubscribeToProperty)->GetFunction());
  target->Set(v8::String::NewSymbol("list_subscribed_properties"),
              v8::FunctionTemplate::
              New(ListSubscribedProperties)->GetFunction());

  // signals
  target->Set(v8::String::NewSymbol("get_signals_description"),
              v8::FunctionTemplate::
              New(GetSignalsDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_signal_description"),
              v8::FunctionTemplate::New(GetSignalDescription)->GetFunction());
  target->Set(v8::String::NewSymbol("get_signals_description_by_class"),
              v8::FunctionTemplate::
              New(GetSignalsDescriptionByClass)->GetFunction());
  target->Set(v8::String::NewSymbol("get_signal_description_by_class"),
              v8::FunctionTemplate::
              New(GetSignalDescriptionByClass)->GetFunction());

  // signal subscription
  target->Set(v8::String::NewSymbol("register_signal_callback"),
              v8::FunctionTemplate::
              New(RegisterSignalCallback)->GetFunction());
  target->Set(v8::String::NewSymbol("subscribe_to_signal"),
              v8::FunctionTemplate::New(SubscribeToSignal)->GetFunction());
  target->Set(v8::String::NewSymbol("unsubscribe_to_signal"),
              v8::FunctionTemplate::New(UnsubscribeToSignal)->GetFunction());
  target->Set(v8::String::NewSymbol("list_subscribed_signals"),
              v8::FunctionTemplate::
              New(ListSubscribedSignals)->GetFunction());
}

NODE_MODULE(switcher, Init)
