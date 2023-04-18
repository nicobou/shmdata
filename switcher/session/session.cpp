/**
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

#include <string>
#include <iostream>
#include <filesystem>
#include <fstream>
#include "./session.hpp"
#include "../switcher.hpp"
#include "../utils/file-utils.hpp"

namespace fs = std::filesystem;

namespace switcher::session {

Session::Session(Switcher* instance) : switcher_(instance) {
  // retrieve current configuration tree
  auto config_tree = this->switcher_->conf<MPtr(&Configuration::get)>();
  // check for $XDG_STATE_HOME variable
  const auto env_xsh = std::getenv("XDG_STATE_HOME");
  // or use recommended fallback as a default
  const fs::path xsh =
      env_xsh ? fs::path(env_xsh) : fs::path(std::getenv("HOME")) / ".local" / "state";
  // set session basedir member to default value
  this->basedir_ = xsh / "switcher" / "sessions";
  // set custom path from configuration
  std::string custom_path = config_tree->branch_get_value("session.path");
  if (!custom_path.empty()) this->basedir_ = fs::path(custom_path);
  // create directories as needed
  if (!fs::is_directory(this->basedir_)) fs::create_directories(this->basedir_);
};

  std::string Session::save_as(const std::string& filename) {
    // retrieve switcher current state
    auto state = this->switcher_->get_state();
    // compute filepath relative to basedir
    fs::path fname = filename, fpath = this->basedir_ / fname.filename();
    // check for json extension
    if (fpath.extension() != ".json") fpath.replace_extension(fs::path(".json"));
    // retrieve current switcher configuration json string
    const std::string serialized = infotree::json::serialize(state.get());
    // write json content to file at `fpath`
    std::ofstream ofs(fpath.c_str());
    ofs << serialized << "\n";
    ofs.close();
    // return filepath
    return fpath;
  };
  
  bool Session::copy(const std::string& src, const std::string& dst) {
    fs::path a = this->basedir_ / fs::path(src).filename(),
             b = this->basedir_ / fs::path(dst).filename();
    // check for json extensions
    if (a.extension() != ".json") a.replace_extension(fs::path(".json"));
    if (b.extension() != ".json") b.replace_extension(fs::path(".json"));
    // check for path existence
    if (!fs::exists(a)) return false;
    // read file content
    const std::string content = fileutils::get_content(a);
    // write content to destination file
    std::ofstream ofs(b);
    ofs << content << "\n";
    ofs.close();
    return true;
  };

  fs::directory_iterator Session::list() { return fs::directory_iterator(this->basedir_); };

  bool Session::remove(const std::string& filename) {
    // remove filename from directory if it exists
    for (const auto& entry : this->list()) {
      auto fname = entry.path().filename();
      if ((fname == filename || fname == filename + ".json")) {
        switcher_->sw_debug("removing session file {}", entry.path().string());
        return fs::remove(entry.path());
      }
    };
    return false;
  };

  bool Session::load(const std::string& filename) {
    // compute file path
    fs::path fpath = this->basedir_ / fs::path(filename).filename();
    if (fpath.extension() != ".json") fpath.replace_extension(fs::path(".json"));
    // check for path existence
    if (!fs::exists(fpath)) return false;
    // read file content
    const std::string content = fileutils::get_content(fpath);
    // parse the session file content and set switcher state
    switcher_->sw_debug("loading state from session file {}", fpath.string());
    return this->switcher_->load_state(infotree::json::deserialize(content).get());
  };

  const std::string Session::read(const std::string& filename) {
    // compute file path
    fs::path fpath = this->basedir_ / fs::path(filename).filename();
    if (fpath.extension() != ".json") fpath.replace_extension(fs::path(".json"));
    // check for path existence, return empty content for inexistant file
    if (!fs::exists(fpath)) {
      switcher_->sw_warning("session file {} was not found", fpath.string());
      return std::string();
    }
    // read file content
    switcher_->sw_debug("reading content from session file {}", fpath.string());
    const std::string content = fileutils::get_content(fpath);
    // return file content
    return content;
  };

  bool Session::write(const std::string& content, const std::string& filename) {
    // compute file path
    fs::path fpath = this->basedir_ / fs::path(filename).filename();
    if (fpath.extension() != ".json") fpath.replace_extension(fs::path(".json"));

    // write file content
    switcher_->sw_debug("writing content to session file {}", fpath.string());
    return fileutils::save(content, fpath);
  };

};  // namespace switcher::session
