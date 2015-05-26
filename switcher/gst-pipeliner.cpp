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
    main_loop_(std2::make_unique<GlibMainLoop>()),
    gst_pipeline_(
        std2::make_unique<GstPipe>(main_loop_->get_main_context(),
                                   std::bind(&GstPipeliner::on_gst_error,
                                             this,
                                             std::placeholders::_1),
                                   [](){})){
  if (!gst_pipeline_) {
    g_warning("error initializing gstreamer pipeline");
    return;
  }
}

GstPipeliner::~GstPipeliner() {
  if (!commands_.empty())
    while (commands_.begin() != commands_.end())
    {
      delete (*commands_.begin())->command;
      if (!g_source_is_destroyed((*commands_.begin())->src))
        g_source_destroy((*commands_.begin())->src);
      commands_.erase(commands_.begin());
    }
}

void GstPipeliner::play(gboolean play) {
  if (play) {
    gst_pipeline_->play(true);
  } else {
    gst_pipeline_->play(false);
  }
}

bool GstPipeliner::seek(gdouble position_in_ms) {
  return gst_pipeline_->seek(position_in_ms);
}


// gboolean GstPipeliner::run_remove_quid(gpointer user_data) {
//   GstPipeliner *context = static_cast<GstPipeliner *>(user_data);
//   QuiddityManager_Impl::ptr manager = context->manager_impl_.lock();
//   if (manager) {
//     // copying in case of self destruction
//     std::list<std::string> tmp = context->quids_to_remove_;
//     context->quids_to_remove_.clear();
//     for (auto &it: tmp) 
//       manager->remove(it);
//   } else {
//     g_warning("cannot recover from error (no manager available)");
//   }
//   return FALSE;
// }

GstElement *GstPipeliner::get_pipeline() {
  return gst_pipeline_->get_pipeline();
}

// void
// GstPipeliner::print_one_tag(const GstTagList */*list*/,
//                             const gchar */*tag*/,
//                             gpointer /*user_data*/) {
//   // int i, num;

//   // num = gst_tag_list_get_tag_size (list, tag);
//   // for (i = 0; i < num; ++i) {
//   //   const GValue *val;

//   //   /* Note: when looking for specific tags, use the g_tag_list_get_xyz() API,
//   //    * we only use the GValue approach here because it is more generic */
//   //   val = gst_tag_list_get_value_index (list, tag, i);
//   //   if (G_VALUE_HOLDS_STRING (val)) {
//   // g_print ("\t%20s : %s\n", tag, g_value_get_string (val));
//   //   } else if (G_VALUE_HOLDS_UINT (val)) {
//   // g_print ("\t%20s : %u\n", tag, g_value_get_uint (val));
//   //   } else if (G_VALUE_HOLDS_DOUBLE (val)) {
//   // g_print ("\t%20s : %g\n", tag, g_value_get_double (val));
//   //   } else if (G_VALUE_HOLDS_BOOLEAN (val)) {
//   // g_print ("\t%20s : %s\n", tag,
//   //  (g_value_get_boolean (val)) ? "true" : "false");
//   //   } else if (GST_VALUE_HOLDS_BUFFER (val)) {
//   // g_print ("\t%20s : buffer of size %u\n", tag,
//   //  GST_BUFFER_SIZE (gst_value_get_buffer (val)));
//   //   } else if (GST_VALUE_HOLDS_DATE (val)) {
//   // g_print ("\t%20s : date (year=%u,...)\n", tag,
//   //  g_date_get_year (gst_value_get_date (val)));
//   //   } else {
//   // g_print ("\t%20s : tag of type '%s'\n", tag, G_VALUE_TYPE_NAME (val));
//   //   }
//   // }
// }

void GstPipeliner::on_gst_error(GstMessage *msg) {
  // on-error-gsource
  GSourceWrapper *gsrc =
      static_cast<GSourceWrapper *>(g_object_get_data(G_OBJECT(msg->src),
                                                      "on-error-gsource"));
  if(nullptr != gsrc) {
    // removing command in order to get it invoked once
    g_object_set_data(G_OBJECT(msg->src),
                      "on-error-gsource",
                      nullptr);
    gsrc->attach(main_loop_->get_main_context());
  }
  
  // on-error-delete
  const char *name =
      (const char *) g_object_get_data(G_OBJECT(msg->src),
                                       "on-error-delete");
  if (nullptr != name) {
    // removing command in order to get it invoked once
    g_object_set_data(G_OBJECT(msg->src),
                      "on-error-delete",
                      (gpointer) nullptr);
    g_warning("FIXME, handle on-error-delete");
    // GstUtils::g_idle_add_full_with_context(main_loop_->get_main_context(),
    //                                        G_PRIORITY_DEFAULT_IDLE,
    //                                        (GSourceFunc)run_remove_quid,
    //                                        (gpointer)this,
    //                                        nullptr);
  }
}

}  // namespace switcher
