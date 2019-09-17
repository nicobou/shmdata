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

#ifndef __SWITCHER_SWITCHER_H__
#define __SWITCHER_SWITCHER_H__

#include <regex>
#include <string>
#include <vector>
#include "./base-logger.hpp"
#include "./configuration.hpp"
#include "./console-logger.hpp"
#include "./gst-initialized.hpp"
#include "./information-tree.hpp"
#include "./make-consultable.hpp"
#include "./quiddity-container.hpp"
#include "./quiddity-factory.hpp"

namespace switcher {
class Switcher : public GstInitialized {
  friend class Bundle;  // access to qcontainer_ and qfactory_
 public:
  using ptr = std::shared_ptr<Switcher>;

  ~Switcher() = default;

  template <typename L = ConsoleLogger, typename... Largs>
  static Switcher::ptr make_switcher(const std::string& name, Largs... args) {
    Switcher::ptr switcher(new Switcher(name, std::make_unique<L>(std::forward<Largs>(args)...)));
    switcher->me_ = switcher;
    return switcher;
  }

  Switcher& operator=(const Switcher&) = delete;
  Switcher(const Switcher&) = delete;
  std::string get_name() const;
  std::string get_switcher_version() const;

  // State
  InfoTree::ptr get_state() const;
  bool load_state(InfoTree* state);
  void reset_state(bool remove_created_quiddities);

  // Configuration
  Make_delegate(Switcher, Configuration, &conf_, conf);

  // Quiddity Factory
  Make_delegate(Switcher, quid::Factory, &qfactory_, factory);

  // Quiddity container
  Make_delegate(Switcher, quid::Container, qcontainer_.get(), quids);

  static std::string get_shm_dir() { return "/tmp"; }
  static std::string get_shm_prefix() { return "switcher_"; }

 private:
  Switcher() = delete;
  template <typename L>  // unique_ptr<LOG_TYPE>
  Switcher(const std::string& name, L&& log)
      : log_(std::move(log)),
        qfactory_(log_.get()),
        conf_(log_.get(),
              [this]() {
                apply_gst_configuration();
                register_bundle_from_configuration();
              }),
        qcontainer_(quid::Container::make_container(this, &qfactory_, log_.get())),
        name_(std::regex_replace(name, std::regex("[^[:alnum:]| ]"), "-")) {
    remove_shm_zombies();
  }
  void apply_gst_configuration();
  void register_bundle_from_configuration();
  void remove_shm_zombies() const;
  static void init_gst();

  mutable std::unique_ptr<BaseLogger> log_;
  quid::Factory qfactory_;
  Configuration conf_;
  quid::Container::ptr qcontainer_;
  std::string name_;
  std::vector<std::string> quiddities_at_reset_{};
  std::weak_ptr<Switcher> me_{};
};
}  // namespace switcher

#endif
