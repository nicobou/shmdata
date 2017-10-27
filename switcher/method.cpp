/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

/**
 * The Method class
 */

#include "./method.hpp"

namespace switcher {
Method::Method() : closure_(nullptr) { json_description_.reset(new JSONBuilder()); }

Method::~Method() {
  if (closure_ != nullptr) g_closure_unref(closure_);
}

Method::Method(const Method& source) { copy_method(source); }

Method& Method::operator=(const Method& source) {
  copy_method(source);
  return *this;
}

void Method::copy_method(const Method& source) {
  long_name_ = source.long_name_;
  method_name_ = source.method_name_;
  short_description_ = source.short_description_;
  return_description_ = source.return_description_;
  arg_description_ = source.arg_description_;
  if (closure_ != nullptr) {
    g_closure_ref(source.closure_);
    closure_ = source.closure_;
  }
  return_type_ = source.return_type_;
  arg_types_ = source.arg_types_;
  num_of_value_args_ = source.num_of_value_args_;
  json_description_ = source.json_description_;
}

BoolLog Method::set_method(method_ptr method,
                           return_type return_type,
                           args_types arg_types,
                           gpointer user_data) {
  if (arg_types.size() < 1) {
    return BoolLog(false, "Method::set_method is called with empty arg_types");
  }
  if (method == nullptr) {
    return BoolLog(false, "Method::set_method is called with a nullptr function");
  }
  closure_ = g_cclosure_new(G_CALLBACK(method), user_data, Method::destroy_data);
  g_closure_set_marshal(closure_, g_cclosure_marshal_generic);
  return_type_ = return_type;
  arg_types_ = arg_types;
  num_of_value_args_ = arg_types_.size();

  return BoolLog(true);
}

bool Method::invoke(std::vector<std::string> args, GValue* result_value) {
  // GValue result_value = G_VALUE_INIT;

  if (args.size() != num_of_value_args_ && arg_types_[0] != G_TYPE_NONE) {
    return false;
  }

  GValue params[arg_types_.size()];

  // with args
  if (arg_types_[0] != G_TYPE_NONE) {
    for (gulong i = 0; i < num_of_value_args_; i++) {
      params[i] = G_VALUE_INIT;
      g_value_init(&params[i], arg_types_[i]);
      if (!gst_value_deserialize(&params[i], args[i].c_str())) {
        return false;
      }
    }
  } else {
    params[0] = G_VALUE_INIT;
    g_value_init(&params[0], G_TYPE_STRING);
    gst_value_deserialize(&params[0], "");
  }

  g_value_init(result_value, return_type_);
  g_closure_invoke(closure_, result_value, num_of_value_args_, params, nullptr);

  for (guint i = 0; i < num_of_value_args_; i++) g_value_unset(&params[i]);
  return true;
}

void Method::destroy_data(gpointer /*data */, GClosure* /*closure */) {}

void Method::set_description(std::string long_name,
                             std::string method_name,
                             std::string short_description,
                             std::string return_description,
                             args_doc arg_description) {
  long_name_ = long_name;
  method_name_ = method_name;
  short_description_ = short_description;
  return_description_ = return_description;
  arg_description_ = arg_description;
}

void Method::make_description() {
  json_description_->reset();
  json_description_->begin_object();
  json_description_->add_string_member("name", long_name_.c_str());
  json_description_->add_string_member("id", method_name_.c_str());
  json_description_->add_string_member("description", short_description_.c_str());
  json_description_->add_string_member("parent", get_category().c_str());
  json_description_->add_int_member("order", get_position_weight());
  json_description_->add_string_member("return type", g_type_name(return_type_));
  json_description_->add_string_member("return description", return_description_.c_str());

  json_description_->set_member_name("arguments");
  json_description_->begin_array();
  args_doc::iterator it;
  if (!arg_description_.empty()) {
    for (auto& it : arg_description_) {
      json_description_->begin_object();
      json_description_->add_string_member("long name", std::get<0>(it).c_str());
      json_description_->add_string_member("name", std::get<1>(it).c_str());
      json_description_->add_string_member("description", std::get<2>(it).c_str());
      json_description_->add_string_member("type",  // FIXME only the first arg ?
                                           g_type_name(arg_types_[0]));
      json_description_->end_object();
    }
  }
  json_description_->end_array();
  json_description_->end_object();
}

// json formated description
std::string Method::get_description() {
  make_description();
  return json_description_->get_string(true);
}

JSONBuilder::Node Method::get_json_root_node() {
  make_description();
  return json_description_->get_root();
}

std::vector<GType> Method::make_arg_type_description(GType first_arg_type, ...) {
  std::vector<GType> res;
  GType arg_type;
  va_list vl;
  va_start(vl, first_arg_type);
  res.push_back(first_arg_type);
  while ((arg_type = va_arg(vl, GType))) res.push_back(arg_type);
  va_end(vl);
  return res;
}

// FIXME, make this more robust to user missing strings
Method::args_doc Method::make_arg_description(const char* first_arg_long_name, ...) {
  args_doc res;
  va_list vl;
  char* arg_name;
  char* arg_desc;
  va_start(vl, first_arg_long_name);
  if (g_strcmp0(first_arg_long_name, "none") != 0 && (arg_name = va_arg(vl, char*)) &&
      (arg_desc = va_arg(vl, char*)))
    res.push_back(std::make_tuple(first_arg_long_name, arg_name, arg_desc));
  gboolean parsing = true;
  do {
    char* arg_long_name = va_arg(vl, char*);
    if (arg_long_name != nullptr) {
      arg_name = va_arg(vl, char*);
      arg_desc = va_arg(vl, char*);
      if (arg_name != nullptr && arg_desc != nullptr)
        res.push_back(std::make_tuple(arg_long_name, arg_name, arg_desc));
      else
        parsing = false;
    } else
      parsing = false;
  } while (parsing);
  va_end(vl);
  return res;
}
}
