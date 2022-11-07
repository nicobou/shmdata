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
 *
 * @file pysession.hpp
 * @brief The `Session Manager` for Switcher.
 *        Allows to create new session files based on `session.path` configuration
 *        key and to copy, list, remove and read existing session files.
 *
 * @author Aurélien Perronneau
 * @date 2021-08-14
 */

#ifndef __PYSWITCH_SESSION_H__
#define __PYSWITCH_SESSION_H__

#include <Python.h>
#include <structmember.h>
#include "switcher/session/session.hpp"

using namespace switcher::session;

/**
 * @brief The `Session` extension type for the `pyquid` package.
 */
class pySession {
 public:
  /**
   * @brief  This is the type object for the extension type.
   *         it is the same object as type in the Python layer.
   */
  using SessionObject = struct {
    PyObject_HEAD PyObject* pyswitch{nullptr};
    Session::ptr csession{nullptr};
    PyInterpreterState* interpreter_state{nullptr};
  };
  /**
   * @brief Structure used to describe methods of the extension type.
   */
  static PyMethodDef methods[];
  /**
   * @brief Structure used to describe the extension type.
   */
  static PyTypeObject pyType;

 private:
  /**
   * @brief A pointer to an instance creation function
   *
   * @param type The type of the object being created
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return The newly created instance
   */
  static PyObject* tp_new(PyTypeObject* type,
                          PyObject* Py_UNUSED(args),
                          PyObject* Py_UNUSED(kwargs));

  /**
   * @brief A pointer to an instance destructor function
   *
   * @param self The instance to be destructed
   *
   * @return void
   */
  static void tp_dealloc(SessionObject* self,
                         PyObject* Py_UNUSED(args),
                         PyObject* Py_UNUSED(kwargs));

  /**
   * @brief A pointer to an instance initialization function
   *
   * @param self The instance to be initialized
   *
   * @return Returns 0 on success, -1 and sets an exception on error.
   */
  static int tp_init(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));

  /**
   * @brief Write Switcher's current state in a new file.
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A Unicode object from a UTF-8 encoded null-terminated char buffer
   *         that holds the absolute path to the newly created file or an empty
   *         path to assert how the copy went.
   */
  static PyObject* save_as(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));

  /**
   * @brief Duplicate an existing session file with a new name
   *        Does not have to be the current session file
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A python boolean asserting how the copy went
   */
  static PyObject* copy(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));

  /**
   * @brief List existing session files in the session directory.
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A python list of existing session files
   */
  static PyObject* list(SessionObject* self,
                        PyObject* Py_UNUSED(args),
                        PyObject* Py_UNUSED(kwargs));

  /**
   * @brief Delete an existing session file without affecting
   *        the current session state.
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A python boolean asserting how the removal went
   */
  static PyObject* remove(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));

  /**
   * @brief Load the contents of a session file on disk, parse it
   *        and set Switcher's state from it.
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A python boolean asserting how the load went
   */
  static PyObject* load(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));

  /**
   * @brief Read the contents of a session file on disk, and return content.
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A python boolean asserting how the read went
   */
  static PyObject* read(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));

  /**
   * @brief Write content to a session file on disk without affecting current 
   * session state
   *
   * @param self The instance of a `SessionObject` C-struct member of pySession
   * @param args Positional arguments passed to the function
   * @param kwargs Keyword arguments passed to the function
   *
   * @return A python boolean asserting how the write went
   */
  static PyObject* write(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs));
};

#endif
