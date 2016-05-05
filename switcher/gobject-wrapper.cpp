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
 * Class wrapping gobject for custum signals
 */

#include "./gobject-wrapper.hpp"
#include <glib/gprintf.h>
#include "./scope-exit.hpp"

namespace switcher {
// gobject
typedef struct _MyObject {
  GObject parent_instance;
  void* context;
} MyObject;

typedef struct _MyObjectClass { GObjectClass parent_class; } MyObjectClass;

static GType my_object_get_type(void);
G_DEFINE_TYPE(MyObject, my_object, G_TYPE_OBJECT);

static void my_object_finalize(GObject* gobject) {
  G_OBJECT_CLASS(my_object_parent_class)->finalize(gobject);
}

static void my_object_class_init(MyObjectClass* klass) {
  GObjectClass* gobject_class = G_OBJECT_CLASS(klass);
  gobject_class->finalize = my_object_finalize;
}

static void my_object_init(MyObject* /*self */) {}

// ---------------------------------- CPP CLASS ----------------------------
// signals
guint GObjectWrapper::next_signal_num_ = 1;
// std::map<guint, GObjectCustomSignal::ptr> GObjectWrapper::custom_signals_;

GObjectWrapper::GObjectWrapper() {
  my_object_ = (MyObject*)g_object_new(my_object_get_type(), nullptr);
  my_object_->context = this;
}

guint GObjectWrapper::make_signal(GType return_type,
                                  guint n_params,
                                  GType* param_types) {
  guint sig_id = next_signal_num_;
  next_signal_num_++;
  gchar* name = g_strdup_printf("custom_signal_%d", sig_id);
  On_scope_exit { g_free(name); };

  // TODO find a way to get CLASS without instanciating an unused object
  MyObject* obj = (MyObject*)g_object_new(my_object_get_type(), nullptr);
  guint signal_id = g_signal_newv(name,
                                  G_TYPE_FROM_CLASS(G_OBJECT_GET_CLASS(obj)),
                                  G_SIGNAL_RUN_LAST,
                                  0,
                                  nullptr,  // GSignalAccumulator
                                  nullptr,  // gpointer accu_data
                                  nullptr,  // GSignalCMarshaller
                                  return_type,
                                  n_params,
                                  param_types);
  g_object_unref(obj);

  return signal_id;
}

GObject* GObjectWrapper::get_gobject() { return G_OBJECT(my_object_); }

GObjectWrapper::~GObjectWrapper() { g_object_unref(my_object_); }

}  // namespace switcher
