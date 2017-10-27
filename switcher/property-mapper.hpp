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

#ifndef __SWITCHER_PROPERTY_MAPPER_H__
#define __SWITCHER_PROPERTY_MAPPER_H__

#include "./quiddity.hpp"

namespace switcher {
class PropertyMapper : public Quiddity {
 public:
  PropertyMapper(QuiddityConfiguration&&);
  ~PropertyMapper();
  PropertyMapper(const PropertyMapper&);
  PropertyMapper& operator=(const PropertyMapper&);

 private:
  std::weak_ptr<Quiddity> source_quiddity_{};
  std::string source_property_name_{};

  std::weak_ptr<Quiddity> sink_quiddity_{};
  // GParamSpec *sink_quiddity_pspec_;
  std::string sink_property_name_{};

  PContainer::register_id_t reg_id_{0};
  PContainer::prop_id_t source_prop_id_{0};
  PContainer::prop_id_t sink_prop_id_{0};
  // scale values (local properties):
  PContainer::prop_id_t sink_min_id_{0};
  PContainer::prop_id_t sink_max_id_{0};
  PContainer::prop_id_t source_min_id_{0};
  PContainer::prop_id_t source_max_id_{0};
  double sink_min_{0};
  double sink_max_{0};
  double source_min_{0};
  double source_max_{0};

  void make_numerical_source_properties();
  void make_numerical_sink_properties();
  void property_updated_cb();
  static gboolean set_source_property_method(gchar* quiddity_name,
                                             gchar* property_name,
                                             void* user_data);
  static gboolean set_sink_property_method(gchar* quiddity_name,
                                           gchar* property_name,
                                           void* user_data);
  void unsubscribe_source_property();
};

}  // namespace switcher
#endif
