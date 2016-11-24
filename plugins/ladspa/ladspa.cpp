/*
 * This file is part of switcher-ladspa.
 *
 * switcher-ladspa is free software; you can redistribute it and/or
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

#include "./ladspa.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(LADSPA);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(LADSPA,
                                     "ladspa",
                                     "LADSPA plugin",
                                     "audio",
                                     "reader/writer",
                                     "Choice of LADSPA plugins",
                                     "LGPL",
                                     "Jérémie Soria");

const std::vector<std::string> LADSPA::KPropertiesBlackList = {"name", "parent"};

LADSPA::LADSPA(const std::string&)
    : shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      plugins_list_(get_ladspa_plugins()),
      plugins_(Selection<>(std::move(plugins_list_.first), std::move(plugins_list_.second), 0)) {
  if (plugins_list_.first.empty()) return;

  plugins_id_ = pmanage<MPtr(&PContainer::make_selection<>)>(
      "plugins",
      [this](size_t val) {
        plugins_.select(val);

        saved_properties_.clear();
        if (!create_gst_pipeline()) return false;
        first_connect_ = true;

        return true;
      },
      [this]() { return plugins_.get(); },
      "LADSPA plugin",
      "Select a LADSPA plugin among all the ones installed on the system.",
      plugins_);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string&) { return on_shmdata_disconnect(); },
      [this]() { return on_shmdata_disconnect(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      1);

  pmanage<MPtr(&PContainer::set_to_current)>(plugins_id_);

  is_valid_ = true;
}

bool LADSPA::init() { return is_valid_; }

LADSPA::PluginList LADSPA::get_ladspa_plugins() {
  PluginList plugins_list;
  auto plugins = GstUtils::element_factory_list_to_pair_of_vectors(
      GST_ELEMENT_FACTORY_TYPE_MEDIA_AUDIO, GST_RANK_NONE, false);
  size_t index = 0;
  for (auto& plugin : plugins.second) {
    if (StringUtils::starts_with(plugin, "ladspa")) {
      plugins_list.first.push_back(plugins.first.at(index));
      plugins_list.second.push_back(plugin);
    }
    ++index;
  }

  if (!plugins.first.size()) {
    g_message("No LADSPA plugin was found.");
    g_warning("No LADSPA plugin was found.");
  }

  return plugins_list;
}

bool LADSPA::create_gst_pipeline() {
  GError* error = nullptr;
  std::string target_format = g_value_get_string(
      GstUtils::get_gst_element_capability(plugins_.get_attached(), "format", GST_PAD_SINK));
  auto target_channels = g_value_get_int(
      GstUtils::get_gst_element_capability(plugins_.get_attached(), "channels", GST_PAD_SINK));

  std::string description =
      std::string("shmdatasrc name=shmsrc ! audioconvert ! capsfilter caps=\"audio/x-raw,format=" +
                  target_format + ",channels=" + std::to_string(target_channels) + "\" ! " +
                  plugins_.get_attached() + " name=ladspa ");

  // We do this in case the pipeline was reset, otherwise the properties values will be reset.
  for (auto& property : saved_properties_) {
    description += std::string(property.first) + "=" + property.second + " ";
  }

  saved_properties_.clear();

  description += "!  shmdatasink name=shmsink sync=false";

  auto new_ladspa_bin = gst_parse_bin_from_description(description.c_str(), TRUE, &error);

  if (error) {
    g_warning(
        "Could not create ladspa plugin %s: %s", plugins_.get_attached().c_str(), error->message);
    g_error_free(error);
    return false;
  }

  auto ladspa_element = gst_bin_get_by_name(GST_BIN(new_ladspa_bin), "ladspa");
  auto shmdatasrc = gst_bin_get_by_name(GST_BIN(new_ladspa_bin), "shmsrc");
  auto shmdatasink = gst_bin_get_by_name(GST_BIN(new_ladspa_bin), "shmsink");

  if (!ladspa_element || !shmdatasrc || !shmdatasink) return false;

  ladspa_element_ = ladspa_element;
  shmdatasrc_ = shmdatasrc;
  shmdatasink_ = shmdatasink;
  ladspa_bin_ = new_ladspa_bin;

  get_gst_properties();

  return true;
}

void LADSPA::get_gst_properties() {
  GParamSpec** property_specs;
  unsigned int num_properties;

  property_specs =
      g_object_class_list_properties(G_OBJECT_GET_CLASS(ladspa_element_), &num_properties);
  On_scope_exit { g_free(property_specs); };

  for (auto& property : properties_) {
    pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>(property));
  }

  properties_.clear();
  for (unsigned int i = 0; i < num_properties; ++i) {
    std::string property_name = property_specs[i]->name;

    if (std::find(LADSPA::KPropertiesBlackList.begin(),
                  LADSPA::KPropertiesBlackList.end(),
                  property_name) != LADSPA::KPropertiesBlackList.end())
      continue;

    properties_.push_back(property_name);
  }

  for (auto& property : properties_) {
    pmanage<MPtr(&PContainer::push)>(property,
                                     GPropToProp::to_prop(G_OBJECT(ladspa_element_), property));
  }
}

bool LADSPA::on_shmdata_connect(const std::string& shmpath) {
  shmpath_ = shmpath;
  shmpath_transformed_ = make_file_name("audio");

  // We get the values before resetting the pipeline before the first connection.
  // After that this will be done during the disconnection event.
  // This is a bit far-fetched but it is necessary due to the fact that we have to create the ladspa
  // element to fetch its properties and we also recreate the pipeline at each connection since it
  // is reset on disconnection. Without this we would lose the property changes between the
  // creation/selection of the plugin and its first connection.
  if (first_connect_) {
    for (auto& prop_name : properties_) {
      saved_properties_[prop_name] = pmanage<MPtr(&PContainer::get_str_str)>(prop_name);
    }
    first_connect_ = false;
  }

  create_gst_pipeline();
  pmanage<MPtr(&PContainer::disable)>(plugins_id_, ShmdataConnector::disabledWhenConnectedMsg);
  shmsrc_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmdatasrc_,
      [this](const std::string& caps) {
        graft_tree(".shmdata.reader." + shmpath_,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.reader." + shmpath_));

  shmsink_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmdatasink_,
      [this](const std::string& caps) {
        graft_tree(".shmdata.writer." + shmpath_transformed_,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.writer." + shmpath_transformed_),
      [this]() { prune_tree(".shmdata.writer." + shmpath_transformed_); });

  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  g_object_set(G_OBJECT(shmdatasrc_), "socket-path", shmpath_.c_str(), nullptr);
  g_object_set(G_OBJECT(shmdatasink_), "socket-path", shmpath_transformed_.c_str(), nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), ladspa_bin_);

  gst_pipeline_->play(true);

  return true;
}

bool LADSPA::on_shmdata_disconnect() {
  pmanage<MPtr(&PContainer::enable)>(plugins_id_);
  shmsrc_sub_.reset();
  shmsink_sub_.reset();

  // Save the properties values if we reconnect the same plugin.
  for (auto& prop_name : properties_) {
    saved_properties_[prop_name] = pmanage<MPtr(&PContainer::get_str_str)>(prop_name);
  }

  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  return true;
}

bool LADSPA::can_sink_caps(std::string str_caps) {
  if (!StringUtils::starts_with(str_caps, "audio/x-raw")) return false;

  GstCaps* caps = gst_caps_from_string(str_caps.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (nullptr == s) {
    g_warning("Cannot get structure from caps (ladspa)");
    return false;
  }

  auto rate = 0;
  if (!gst_structure_get_int(s, "rate", &rate)) {
    g_warning("Cannot get rate from shmdata description (ladspa)");
    return false;
  }

  auto target_rate =
      GstUtils::get_gst_element_capability_as_range(plugins_.get_attached(), "rate", GST_PAD_SINK);

  return ((target_rate.first <= rate && rate <= target_rate.second));
}
};
