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

#ifndef __SWITCHER_BUNDLE_H__
#define __SWITCHER_BUNDLE_H__

#include <atomic>
#include <vector>
#include "../../shmdata/connector.hpp"
#include "../../switcher.hpp"
#include "../quiddity.hpp"
#include "../startable.hpp"
#include "./description-parser.hpp"

namespace switcher {
namespace quiddity {
namespace bundle {
class Bundle : public Quiddity, public Startable {
 public:
  Bundle(Config&&);
  ~Bundle();
  Bundle(const Bundle&) = delete;
  Bundle& operator=(const Bundle&) = delete;

  std::string make_shmpath(const std::string& suffix) const final;

 private:
  struct on_tree_data_t {
    on_tree_data_t(Bundle* self,
                   const std::string& quid_name,
                   Quiddity* quid,
                   const bundle::quiddity_spec_t& spec)
        : self_(self), quid_name_(quid_name), quid_(quid), quid_spec_(spec) {}
    Bundle* self_;
    std::string quid_name_;
    Quiddity* quid_;
    bundle::quiddity_spec_t quid_spec_;
  };

 private:
  quiddity::Config conf_;
  std::atomic_bool quitting_{false};
  std::vector<std::pair<std::string /*quid_name*/, std::string /*shmpath*/>> connected_shms_{};
  std::mutex connected_shms_mtx_{};
  std::vector<std::string> start_quids_{};
  std::vector<std::string> exposed_writer_quids_{};
  std::string reader_quid_{};
  shmdata::Connector shmcntr_;
  std::vector<std::unique_ptr<on_tree_data_t>> on_tree_datas_{};
  std::string pipeline_{};
  Switcher::ptr manager_;
  std::vector<unsigned int> quiddity_removal_cb_ids_{};

  bool start() final;
  bool stop() final;
  bool make_quiddities(const std::vector<bundle::quiddity_spec_t>& quids);
  static void on_tree_grafted(const std::string& key, void* user_data);
  static void on_tree_pruned(const std::string& key, void* user_data);
  static bool property_is_displayed(bundle::quiddity_spec_t quid_spec, std::string property_name);
};

// wrappers for the abstract factory registration
Quiddity* create(Config&& conf);
void destroy(Quiddity* quiddity);
}  // namespace bundle
}  // namespace quiddity
}  // namespace switcher
#endif
