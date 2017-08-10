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

#include "./property-mapper.hpp"
#include <math.h>
#include "./gst-utils.hpp"
#include "./quiddity-container.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PropertyMapper,
                                     "property-mapper",
                                     "Switcher Property Mapper",
                                     "utils",
                                     "",
                                     "map two properties, one being slave of the other",
                                     "LGPL",
                                     "Nicolas Bouillot");

PropertyMapper::PropertyMapper(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)) {
  install_method("Set Source Property",                                // long name
                 "set-source-property",                                // name
                 "set the master property",                            // description
                 "success of fail",                                    // return description
                 Method::make_arg_description("Quiddity Name",         // first arg long name
                                              "quiddity_name",         // fisrt arg name
                                              "Name of the quiddity",  // first arg description
                                              "property Name",         // first arg long name
                                              "property_name",         // fisrt arg name
                                              "Name of the property",  // first arg description
                                              nullptr),
                 (Method::method_ptr)&set_source_property_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);

  install_method("Set Sink Property",                                  // long name
                 "set-sink-property",                                  // name
                 "set the slave property",                             // description
                 "success of fail",                                    // return description
                 Method::make_arg_description("Quiddity Name",         // first arg long name
                                              "quiddity_name",         // fisrt arg name
                                              "Name of the quiddity",  // first arg description
                                              "property Name",         // first arg long name
                                              "property_name",         // fisrt arg name
                                              "Name of the property",  // first arg description
                                              nullptr),
                 (Method::method_ptr)&set_sink_property_method,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);
}

PropertyMapper::~PropertyMapper() { unsubscribe_source_property(); }

void PropertyMapper::unsubscribe_source_property() {
  Quiddity::ptr source_quid = source_quiddity_.lock();
  if ((bool)source_quid) {
    source_quid->prop<MPtr(&PContainer::unsubscribe)>(source_prop_id_, reg_id_);
  }
}

gboolean PropertyMapper::set_source_property_method(gchar* quiddity_name,
                                                    gchar* property_name,
                                                    void* user_data) {
  PropertyMapper* context = static_cast<PropertyMapper*>(user_data);
  Quiddity::ptr quid = context->qcontainer_->get_quiddity(quiddity_name);
  if (!(bool)quid) {
    context->debug("quiddity % not found", std::string(quiddity_name));
    return FALSE;
  }
  context->source_prop_id_ = quid->prop<MPtr(&PContainer::get_id)>(property_name);
  if (0 == context->source_prop_id_) {
    context->debug("property % not found", std::string(property_name));
    return FALSE;
  }
  if (!quid->tree<MPtr(&InfoTree::branch_has_data)>(std::string("property.") + property_name +
                                                    ".min")) {
    context->debug("property % has no min/max defined", std::string(property_name));
    return FALSE;
  }
  // unsubscribing previously registered property
  context->unsubscribe_source_property();
  context->source_quiddity_ = quid;
  context->source_property_name_ = property_name;

  context->source_min_ = quid->tree<MPtr(&InfoTree::branch_read_data<double>)>(
      std::string("property.") + property_name + ".min");
  context->source_max_ = quid->tree<MPtr(&InfoTree::branch_read_data<double>)>(
      std::string("property.") + property_name + ".max");
  context->make_numerical_source_properties();

  context->reg_id_ = quid->prop<MPtr(&PContainer::subscribe)>(
      context->source_prop_id_, [context]() { context->property_updated_cb(); });
  auto source_tree = context->prune_tree(".source", false);
  if (!source_tree) source_tree = InfoTree::make();
  source_tree->graft(".quiddity", InfoTree::make(std::string(quiddity_name)));
  source_tree->graft(".property", InfoTree::make(std::string(property_name)));
  context->graft_tree(".source.", source_tree);
  return TRUE;
}

void PropertyMapper::make_numerical_source_properties() {
  pmanage<MPtr(&PContainer::remove)>(source_min_id_);
  source_min_id_ =
      pmanage<MPtr(&PContainer::make_double)>("source-min",
                                              [this](double val) {
                                                source_min_ = val;
                                                return true;
                                              },
                                              [this]() { return source_min_; },
                                              "Source Property Minimum",
                                              "The minimum value to be applyed when mapping",
                                              source_min_,
                                              std::numeric_limits<double>::lowest(),
                                              std::numeric_limits<double>::max());

  pmanage<MPtr(&PContainer::remove)>(source_max_id_);
  pmanage<MPtr(&PContainer::make_double)>("source-max",
                                          [this](double val) {
                                            source_max_ = val;
                                            return true;
                                          },
                                          [this]() { return source_max_; },
                                          "Source Property Maximum",
                                          "The maximmum value to be applyed when mapping",
                                          source_max_,
                                          std::numeric_limits<double>::lowest(),
                                          std::numeric_limits<double>::max());
}

void PropertyMapper::make_numerical_sink_properties() {
  pmanage<MPtr(&PContainer::remove)>(sink_min_id_);
  sink_min_id_ =
      pmanage<MPtr(&PContainer::make_double)>("sink-min",
                                              [this](double val) {
                                                sink_min_ = val;
                                                return true;
                                              },
                                              [this]() { return sink_min_; },
                                              "Sink Property Minimum",
                                              "The minimum value to be applyed when mapping",
                                              sink_min_,
                                              std::numeric_limits<double>::lowest(),
                                              std::numeric_limits<double>::max());

  pmanage<MPtr(&PContainer::remove)>(sink_max_id_);
  pmanage<MPtr(&PContainer::make_double)>("sink-max",
                                          [this](double val) {
                                            sink_max_ = val;
                                            return true;
                                          },
                                          [this]() { return sink_max_; },
                                          "Sink Property Maximum",
                                          "The maximmum value to be applyed when mapping",
                                          sink_max_,
                                          std::numeric_limits<double>::lowest(),
                                          std::numeric_limits<double>::max());
}

void PropertyMapper::property_updated_cb() {
  // return if not property to update
  Quiddity::ptr sinkquid = sink_quiddity_.lock();
  if (!(bool)sinkquid) return;

  Quiddity::ptr sourcequid = source_quiddity_.lock();
  if (!(bool)sourcequid) return;

  sourcequid->prop<MPtr(&PContainer::update_value_in_tree)>(source_prop_id_);
  double val = sourcequid->tree<MPtr(&InfoTree::branch_read_data<double>)>(
      std::string("property.") + source_property_name_ + ".value");

  // scaling and transforming the value into a gdouble value
  double transformed_val =
      ((double)val - source_min_) * (sink_max_ - sink_min_) / (source_max_ - source_min_) +
      sink_min_;

  sinkquid->prop<MPtr(&PContainer::set_str)>(sink_prop_id_, std::to_string(transformed_val));
}

gboolean PropertyMapper::set_sink_property_method(gchar* quiddity_name,
                                                  gchar* property_name,
                                                  void* user_data) {
  PropertyMapper* context = static_cast<PropertyMapper*>(user_data);
  Quiddity::ptr quid = context->qcontainer_->get_quiddity(quiddity_name);
  if (!(bool)quid) {
    context->debug("quiddity % not found", std::string(quiddity_name));
    return FALSE;
  }
  context->sink_prop_id_ = quid->prop<MPtr(&PContainer::get_id)>(property_name);
  if (0 == context->sink_prop_id_) {
    context->debug("property % not found", std::string(property_name));
    return FALSE;
  }
  if (!quid->tree<MPtr(&InfoTree::branch_has_data)>(std::string("property.") + property_name +
                                                    ".min")) {
    context->debug("property % has no min/max defined", std::string(property_name));
    return FALSE;
  }
  context->sink_min_ = quid->tree<MPtr(&InfoTree::branch_read_data<double>)>(
      std::string("property.") + property_name + ".min");
  context->sink_max_ = quid->tree<MPtr(&InfoTree::branch_read_data<double>)>(
      std::string("property.") + property_name + ".max");
  context->make_numerical_sink_properties();
  context->sink_quiddity_ = quid;
  context->sink_property_name_ = property_name;
  auto sink_tree = context->prune_tree(".sink", false);
  if (!sink_tree) sink_tree = InfoTree::make();
  sink_tree->graft(".quiddity", InfoTree::make(std::string(quiddity_name)));
  sink_tree->graft(".property", InfoTree::make(std::string(property_name)));
  context->graft_tree(".sink", sink_tree);
  return TRUE;
}

}  // namespace switcher
