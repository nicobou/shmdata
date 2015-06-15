/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "switcher/scope-exit.hpp"

// note: do not write multiple "On_scope_exit" call in the same line
int
main() {
  int counter = 0;
  {
    counter = 1;
    On_scope_exit {
      counter = 2;
    };
    On_scope_exit {
    };  // ensuring multiple calls in the same scope
  }

  On_scope_exit {
    counter = 3;
  };  // should be assigned after "main"

  if (2 == counter)
    return 0;
  return 1;
}
