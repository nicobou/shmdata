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

#ifndef __SWITCHER_SESSION_H__
#define __SWITCHER_SESSION_H__

#include <filesystem>
#include <list>
#include <string>
#include <memory>

#include "../configuration/configuration.hpp"
#include "../infotree/information-tree.hpp"
#include "../logger/logger.hpp"
#include "../utils/any.hpp"
#include "../utils/make-consultable.hpp"

namespace fs = std::filesystem;

namespace switcher {
  class Switcher;

  namespace session {

  /**
   * @brief The `Session` class
   *
   *        This class allows Switcher to manage session files to keep a
   *        history of the current state of a Switcher session.
   *
   *        Available configuration keys:
   *
   *        "session": {
   *          "path": "/path/to/session_base_directory"
   *        }
   *
   *        At run time, it is guaranteed to be initialized once.
   */
  class Session {
   public:
    using ptr = std::shared_ptr<Session>;
    /**
     * @brief Constructor
     *
     *        Deduces the `session base directory` where we want session files
     *        to be saved following the `XDG Base Directory Specification` and
     *        overrides it with the configured `session.path` key of Switcher
     *        if any.
     *
     *        In case the `session base directory` does not exist on the system,
     *        the constructor takes care to create it as a convenience.
     *
     *        Removal of previously used directories is at the discretion of
     *        the user.
     *
     *        see https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html
     *
     * @param instance A pointer to an instance of Switcher
     */
    Session(Switcher* instance);
    Session() = delete;

    /**
     * @brief Write Switcher's current state in a new file
     *
     * @param filename The name for the newly created file
     *
     * @return An absolute path to the saved session file
     */
    std::string save_as(const std::string& filename);

    /**
     * @brief Duplicate an existing session file with a new name
     *        Does not have to be the current session file
     *
     * @param src The name of the source session file to copy
     * @param dst The name of the destination session file to copy the source's content to
     *
     * @return A boolean asserting how the copy went
     */
    bool copy(const std::string& src, const std::string& dst);

    /**
     * @brief List the existing session files in the session directory
     *
     * @return A directory iterator over files within the session base directory
     */
    fs::directory_iterator list();

    /**
     * @brief Delete an existing session file without affecting the current session state
     *
     * @param filename The name of the file to remove
     *
     * @return A boolean asserting how the removal went
     */
    bool remove(const std::string& filename);

    /**
     * @brief Load a session file on disk, parse it and set Switcher's state from it
     *
     * @param filename The name of the file to load
     *
     * @return A boolean asserting how the load went
     */
    bool load(const std::string& filename);

   /**
     * @brief Read the contents of a session file on disk, and return content
     *
     * @param filename The name of the file to read
     *
     * @return content of session file
     */
    const std::string read(const std::string& filename);

   /**
     * @brief Write content to a session file on disk without affecting current
     * session state
     *
     * @param content content to write to session file
     * @param filename The name of the file to write to
     *
     * @return A boolean asserting how the write went
     */
    bool write(const std::string& content, const std::string& filename);

   private:
    /**
     * @brief An absolute path to the `session base directory` under which
     *        to save session files.
     *
     *        The `session` object from the configuration of Switcher has
     *        a `path` attribute that should point to an absolute path on
     *        the system where we want `session files` to be saved.
     */
    fs::path basedir_;

    /**
     * @brief Pointer to a Switcher instance
     */
    Switcher* switcher_;

    std::shared_ptr<spdlog::logger> logger_;

  };  // class Session
  }; // namesepace session
}; // namesepace switcher

#endif

