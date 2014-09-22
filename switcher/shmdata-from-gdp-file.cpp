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

#include <iostream>

#include <glib.h>
#include <glib/gstdio.h>

#include "./shmdata-from-gdp-file.hpp"
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataFromGDPFile,
                                     "Shmdata File Player",
                                     "shmdata file player",
                                     "play file(s) recorded with shmdatatofile",
                                     "LGPL",
                                     "shmfromfilesource", "Nicolas Bouillot, Emmanuel Durand");

ShmdataFromGDPFile::ShmdataFromGDPFile():custom_prop_(new CustomPropertyHelper()),
                                                      shmdata_names_() {
}

ShmdataFromGDPFile::~ShmdataFromGDPFile() {
}

bool ShmdataFromGDPFile::init_gpipe() {
  init_startable(this);
  init_segment(this);
  install_play_pause();

  input_prefix_param_ = custom_prop_->make_string_property("filename_prefix",
                          "Prefix of the files to look for",
                          input_prefix_.c_str(),
                          (GParamFlags) G_PARAM_READWRITE,
                          ShmdataFromGDPFile::set_input_prefix,
                          ShmdataFromGDPFile::get_input_prefix,
                          this);
  install_property_by_pspec(custom_prop_->get_gobject(),
                            input_prefix_param_,
                            "filename_prefix",
                            "Prefix of the files to look for");

  return true;
}

const gchar *
ShmdataFromGDPFile::get_input_prefix(void *user_data) {
  ShmdataFromGDPFile *ctx = (ShmdataFromGDPFile*)user_data;
  return ctx->input_prefix_.c_str();
}

void
ShmdataFromGDPFile::set_input_prefix(const gchar *prefix, void *user_data) {
  ShmdataFromGDPFile *ctx = (ShmdataFromGDPFile *) user_data;
  if (prefix != nullptr)
    ctx->input_prefix_ = prefix;
}

bool
ShmdataFromGDPFile::start() {
  if (is_started())
    return false;

  make_players();

  return true;
}

bool
ShmdataFromGDPFile::stop() {
  if (!is_started())
    return false;

  clean_players();
  return true;
}

bool ShmdataFromGDPFile::make_players() {
  reset_bin();

  shmdata_names_ = getFilenames(input_prefix_);
  for (auto& it : shmdata_names_) {
    GError *error = nullptr;
    gchar *pipe = g_strdup_printf("filesrc location=%s ! gdpdepay ! identity sync=true",
                                  it.first.c_str());
    On_scope_exit{ g_free(pipe); };
    GstElement *reader_bin = gst_parse_bin_from_description(pipe, TRUE, &error);

    if (error != nullptr) {
      g_warning("ShmdataFromGDPFile - %s", error->message);
      return false;
    }
    g_object_set(G_OBJECT(reader_bin), "async-handling", TRUE, nullptr);
    GstPad *src_pad = gst_element_get_static_pad(reader_bin, "src");
    On_scope_exit{ gst_object_unref(src_pad); };

    gst_bin_add(GST_BIN(bin_), reader_bin);

    ShmdataWriter::ptr writer = std::make_shared<ShmdataWriter>();
    writer->set_path(make_file_name("shmfromfile" + std::to_string(shm_counter_)));
    shm_counter_++;
    writer->plug(bin_, src_pad);
    register_shmdata(writer);

    GstUtils::sync_state_with_parent(reader_bin);
  }

  return true;
}

bool ShmdataFromGDPFile::clean_players() {
  reset_bin();

  clear_shmdatas();
  shm_counter_ = 0;
  
  return true;
}

std::map<std::string, std::string> ShmdataFromGDPFile::getFilenames(std::string prefix) {
  std::map<std::string, std::string> filenames {};
  std::string dirName {};
  std::string prefixName {};

  dirName = prefix.substr(0, prefix.rfind("/"));
  prefixName = prefix.substr(prefix.rfind("/") + 1);
  if (prefix.rfind("/") == std::string::npos)
    dirName = ".";
  else if (prefix[0] != '/')
    dirName = "./" + dirName;

  GDir* dir = g_dir_open(dirName.c_str(), 0, NULL);
  if (dir == NULL) {
    g_warning("Cannot open the directory %s", dirName.c_str());
    return filenames;
  }

  char* file = const_cast<char*>(g_dir_read_name(dir));
  while (file != NULL) {
    if (std::string(file).rfind(prefixName) != std::string::npos) {
      std::string filename = dirName + "/" + std::string(file);
      std::string shmname = std::string(file).substr(std::string(file).rfind(prefixName) + prefixName.size());
      filenames[filename] = shmname;
    }
    file = const_cast<char*>(g_dir_read_name(dir));
  }

  g_dir_close(dir);

  return filenames;
}

// FIXME use signals in switcher for handling gstsrc's eos
// void
// ShmdataFromGDPFile::rewind (gpointer user_data)
// {
//   ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile *>(user_data);
//   context->set_playing (FALSE, context);
// }

// gboolean
// ShmdataFromGDPFile::event_probe_cb (GstPad *pad, GstEvent *event, gpointer user_data)
// {
//   ShmdataFromGDPFile *context = static_cast<ShmdataFromGDPFile *>(user_data);
//   if (GST_EVENT_TYPE (event) == GST_EVENT_EOS) {
//     g_print ("EOS caught and disabled \n");
//     g_print ("----- pad with EOS %s:%s, src: %p %s\n",
//             GST_DEBUG_PAD_NAME (pad),GST_EVENT_SRC(event), gst_element_get_name (GST_EVENT_SRC(event)));
//     g_idle_add ((GSourceFunc) ShmdataFromGDPFile::rewind,
//        (gpointer)context);
//     return FALSE;
//   }
//   return TRUE;
// }
}
