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

#ifndef __SWITCHER_GPIPE_H__
#define __SWITCHER_GPIPE_H__

#include <gst/gst.h>
#include <memory>
#include <vector>
#include <mutex>
#include "./quiddity.hpp"
#include "./segment.hpp"
#include "./gst-pipe.hpp"

namespace switcher {
class Quiddity;
class QuiddityCommand;
class CustomPropertyHelper;
class DecodebinToShmdata;

class GstPipeliner: public Quiddity, public Segment {
  friend DecodebinToShmdata;

 public:
  GstPipeliner();
  virtual ~GstPipeliner();
  GstPipeliner(const GstPipeliner &) = delete;
  GstPipeliner &operator=(const GstPipeliner &) = delete;
  bool init() final;
  virtual bool init_gpipe() = 0;

 protected:
  GstElement *get_bin();
  bool reset_bin();
  GstElement *get_pipeline();
  void install_play_pause();
  void install_seek();
  void install_speed();
  void play(gboolean);  // FIXME use bool
  bool seek(gdouble position_in_ms);

 private:
    typedef struct {
    GstPipeliner *self;
    QuiddityCommand *command;
    GSource *src;
  } QuidCommandArg;

  std::unique_ptr<GstPipe> gst_pipeline_{};
  GstElement *bin_{nullptr}; // FIXME should be private

  //GSource *position_tracking_source_ {nullptr};

  std::unique_ptr<CustomPropertyHelper> gpipe_custom_props_;
  GParamSpec *play_pause_spec_ {nullptr};
  bool play_ {true};
  GParamSpec *seek_spec_ {nullptr};
  gdouble seek_ {0.0};

  std::vector<QuidCommandArg *>commands_ {};

  void on_gst_error(GstMessage *msg);
  
  void make_bin();
  void clean_bin();
  bool speed(gdouble speed);
  static gboolean get_play(void *user_data);
  static void set_play(gboolean play, void *user_data);
  static void set_seek(gdouble position, void *user_data);
  static gdouble get_seek(void *user_data);
  static gboolean speed_wrapped(gdouble speed, gpointer user_data);
  static gboolean run_command(gpointer user_data);
  static void print_one_tag(const GstTagList *list,
                            const gchar *tag, gpointer user_data);
};
}  // namespace switcher

#endif
