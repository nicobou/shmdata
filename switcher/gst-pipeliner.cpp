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
 * The GstPipeliner class
 */

#include <shmdata/base-reader.h>
#include <gst/interfaces/xoverlay.h>
#include <algorithm>
#include "./gst-pipeliner.hpp"
#include "./quiddity.hpp"
#include "./custom-property-helper.hpp"
#include "./gst-utils.hpp"
#include "./quiddity-command.hpp"
#include "./quiddity-manager-impl.hpp"
#include "./scope-exit.hpp"
#include "./std2.hpp"
#include "./g-source-wrapper.hpp"

namespace switcher {
GstPipeliner::GstPipeliner():
    bin_("bin"),
    gpipe_custom_props_(std2::make_unique<CustomPropertyHelper>()) {
  init_segment(this);
}

bool GstPipeliner::init() {
  if (nullptr == get_g_main_context()) {
    g_warning("%s: g_main_context is nullptr", __FUNCTION__);
    return false;
  }
  gst_pipeline_ = std2::make_unique<GstPipe>(get_g_main_context());
  gst_pipeline_->set_on_error_function(std::bind(&GstPipeliner::on_gst_error,
                                                 this,
                                                 std::placeholders::_1));
  make_bin();
  play_pause_spec_ = gpipe_custom_props_->
      make_boolean_property("play",
                            "play",
                            TRUE,
                            (GParamFlags)G_PARAM_READWRITE,
                            GstPipeliner::set_play,
                            GstPipeliner::get_play,
                            this);
  seek_spec_ = gpipe_custom_props_->
      make_double_property("seek", "seek (in percent)",
                           0.0, 1.0, 0.0,
                           (GParamFlags)G_PARAM_READWRITE,
                           GstPipeliner::set_seek,
                           GstPipeliner::get_seek,
                           this);
  return init_gpipe();
}

GstPipeliner::~GstPipeliner() {
  GstUtils::wait_state_changed(gst_pipeline_->get_pipeline());
  clear_shmdatas();
  if (!commands_.empty())
    while (commands_.begin() != commands_.end())
    {
      delete (*commands_.begin())->command;
      if (!g_source_is_destroyed((*commands_.begin())->src))
        g_source_destroy((*commands_.begin())->src);
      commands_.erase(commands_.begin());
    }
  // if (nullptr != position_tracking_source_)
  //   g_source_destroy(position_tracking_source_);
}

void GstPipeliner::install_play_pause() {
  install_property_by_pspec(gpipe_custom_props_->get_gobject(),
                            play_pause_spec_, "play", "Play");
}

void GstPipeliner::install_seek() {
  install_property_by_pspec(gpipe_custom_props_->get_gobject(),
                            seek_spec_, "seek", "Seek");
}

void GstPipeliner::install_speed() {
  install_method("Speed",
                 "speed",
                 "controle speed of pipeline",
                 "success or fail",
                 Method::make_arg_description("Speed",
                                              "speed",
                                              "1.0 is normal speed, 0.5 is half the speed and 2.0 is double speed",
                                              nullptr),
                 (Method::method_ptr) &speed_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_DOUBLE,
                                                   nullptr), this);
}

void GstPipeliner::set_play(gboolean play, void *user_data) {
  GstPipeliner *context = static_cast<GstPipeliner *>(user_data);
  context->play(play);
}

gboolean GstPipeliner::get_play(void *user_data) {
  GstPipeliner *context = static_cast<GstPipeliner *>(user_data);
  return context->play_;
}

void GstPipeliner::play(gboolean play) {
  if (play) {
    gst_pipeline_->play(true);
    play_ = true;
  } else {
    gst_pipeline_->play(false);
    play_ = false;
  }
  gpipe_custom_props_->notify_property_changed(play_pause_spec_);
}

bool GstPipeliner::seek(gdouble position_in_ms) {
  return gst_pipeline_->seek(position_in_ms);
}


gdouble GstPipeliner::get_seek(void *user_data) {
  GstPipeliner *context = static_cast<GstPipeliner *>(user_data);
  return context->seek_;
}

void GstPipeliner::set_seek(gdouble position, void *user_data) {
  GstPipeliner *context = static_cast<GstPipeliner *>(user_data);
  context->seek_ = position;
  context->gst_pipeline_->seek(position);
}

gboolean GstPipeliner::speed_wrapped(gdouble speed, gpointer user_data) {
  GstPipeliner *context = static_cast<GstPipeliner *>(user_data);
  g_debug("speed_wrapped %f", speed);
  if (context->gst_pipeline_->speed(speed))
    return TRUE;
  else
    return FALSE;
}

gboolean GstPipeliner::run_command(gpointer user_data) {
  QuidCommandArg *context = static_cast<QuidCommandArg *>(user_data);
  QuiddityManager_Impl::ptr manager = context->self->manager_impl_.lock();
  if ((bool) manager && context->command != nullptr) {
    switch (context->command->id_) {
      case QuiddityCommand::remove:
        manager->remove(context->command->args_[0]);
        break;
      case QuiddityCommand::invoke:
        {
          manager->invoke(context->command->args_[0],
                          context->command->args_[1],
                          nullptr,
                          context->command->vector_arg_);
        }
        break;
      case QuiddityCommand::set_property:
        {
          manager->set_property(context->command->args_[0],
                                context->command->args_[1],
                                context->command->vector_arg_[0]);
        }
        break;

      default:
        g_debug("on-error-command: %s not implemented\n",
                QuiddityCommand::get_string_from_id(context->command->id_));
    }
  }
  else
    g_warning("GstPipeliner::bus_sync_handler, cannot run command");
  
  auto it = std::find(context->self->commands_.begin(),
                      context->self->commands_.end(),
                      context);
  if (context->self->commands_.end() != it)
  {
    // it->src will be freed by the glib
    //delete (*it)->command;
    context->self->commands_.erase(it);
  }
  return FALSE;  // do not repeat run_command
}

GstElement *GstPipeliner::get_pipeline() {
  return gst_pipeline_->get_pipeline();
}

void
GstPipeliner::print_one_tag(const GstTagList */*list*/,
                            const gchar */*tag*/,
                            gpointer /*user_data*/) {
  // int i, num;

  // num = gst_tag_list_get_tag_size (list, tag);
  // for (i = 0; i < num; ++i) {
  //   const GValue *val;

  //   /* Note: when looking for specific tags, use the g_tag_list_get_xyz() API,
  //    * we only use the GValue approach here because it is more generic */
  //   val = gst_tag_list_get_value_index (list, tag, i);
  //   if (G_VALUE_HOLDS_STRING (val)) {
  // g_print ("\t%20s : %s\n", tag, g_value_get_string (val));
  //   } else if (G_VALUE_HOLDS_UINT (val)) {
  // g_print ("\t%20s : %u\n", tag, g_value_get_uint (val));
  //   } else if (G_VALUE_HOLDS_DOUBLE (val)) {
  // g_print ("\t%20s : %g\n", tag, g_value_get_double (val));
  //   } else if (G_VALUE_HOLDS_BOOLEAN (val)) {
  // g_print ("\t%20s : %s\n", tag,
  //  (g_value_get_boolean (val)) ? "true" : "false");
  //   } else if (GST_VALUE_HOLDS_BUFFER (val)) {
  // g_print ("\t%20s : buffer of size %u\n", tag,
  //  GST_BUFFER_SIZE (gst_value_get_buffer (val)));
  //   } else if (GST_VALUE_HOLDS_DATE (val)) {
  // g_print ("\t%20s : date (year=%u,...)\n", tag,
  //  g_date_get_year (gst_value_get_date (val)));
  //   } else {
  // g_print ("\t%20s : tag of type '%s'\n", tag, G_VALUE_TYPE_NAME (val));
  //   }
  // }
}


void GstPipeliner::make_bin() {
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  {  // reseting the pipeline too
    std::unique_ptr<GstPipe> tmp;
    std::swap(gst_pipeline_, tmp);
  }
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  g_print("+++++++++++++++ maincontext is %p\n", get_g_main_context());
  gst_pipeline_ = std2::make_unique<GstPipe>(get_g_main_context());
g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  
  gst_pipeline_->set_on_error_function(std::bind(&GstPipeliner::on_gst_error,
                                                 this,
                                                 std::placeholders::_1));
    g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
g_object_set(G_OBJECT(bin_.get_raw()), "async-handling", TRUE, nullptr);
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), bin_.get_raw());
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  GstUtils::wait_state_changed(gst_pipeline_->get_pipeline());
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  GstUtils::sync_state_with_parent(bin_.get_raw());
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
  GstUtils::wait_state_changed(bin_.get_raw());
  g_print("ksjdfg %s %d\n", __FUNCTION__, __LINE__);
}

void GstPipeliner::clean_bin() {
  UGstElem tmp("bin");
  if(!tmp)
    return;
  bin_ = std::move(tmp);
}

GstElement *GstPipeliner::get_bin() {
  return bin_.get_raw();
}

bool GstPipeliner::reset_bin() {
  g_print("blblalblalb %s %d\n", __FUNCTION__, __LINE__);
  clean_bin();
  g_print("blblalblalb %s %d\n", __FUNCTION__, __LINE__);
  make_bin();
  g_print("blblalblalb %s %d\n", __FUNCTION__, __LINE__);
  return true;
}

void GstPipeliner::on_gst_error(GstMessage *msg) {
  {
    GSourceWrapper *gsrc =
        (GSourceWrapper *) g_object_get_data(G_OBJECT(msg->src),
                                             "on-error-gsource");
    g_print("%s %p\n", __FUNCTION__, gsrc);
    if(nullptr != gsrc) {
      // removing command in order to get it invoked once
      g_object_set_data(G_OBJECT(msg->src),
                        "on-error-gsource",
                        (gpointer) nullptr);
      gsrc->attach(get_g_main_context());

    }
    
  }
  

  { // FIXME REMOVE on-error-command
    QuiddityCommand *command =
        (QuiddityCommand *) g_object_get_data(G_OBJECT(msg->src),
                                              "on-error-command");
    // removing command in order to get it invoked once
    g_object_set_data(G_OBJECT(msg->src),
                      "on-error-command", (gpointer) nullptr);
    if (command != nullptr) {
      g_debug("error contains data (on-error-command) ");
      QuidCommandArg *args = new QuidCommandArg();
      args->self = this;
      args->command = command;
      args->src = nullptr;
      if (command->time_ > 1) {
        args->src = g_timeout_source_new((guint) command->time_);
        g_source_set_callback(args->src,
                              (GSourceFunc) run_command,
                              args,
                              nullptr);
        commands_.push_back(args);
        g_source_attach(args->src, get_g_main_context());
        g_source_unref(args->src);
      } else {
        GstUtils::g_idle_add_full_with_context(get_g_main_context (),
                                               G_PRIORITY_DEFAULT_IDLE,
                                               (GSourceFunc) run_command,
                                               (gpointer) args,
                                               nullptr);
      }
    }
  }
}

}  // namespace switcher
