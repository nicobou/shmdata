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
 */

#ifndef __SWITCHER_PYQUIDDITIES_H__
#define __SWITCHER_PYQUIDDITIES_H__

#include <Python.h>

#include "./pyswitch.hpp"

#include "switcher/switcher.hpp"
using namespace switcher;

class pyQuiddities {
public:
  using pyQuidditiesObject = struct {
    PyObject_HEAD PyObject* pyswitch{nullptr};
  };

  /**
   * @brief Structure that defines the python type of the pyQuiddities object.
   * @see https://docs.python.org/3/c-api/typeobj.html#type-objects in order to
   *      understand the implementation patterns.
   */
  static PyTypeObject pyType;

  /**
   * @brief Structure that contains all the specific sequence methods.
   * @see https://docs.python.org/3/c-api/typeobj.html#c.PyTypeObject.tp_as_sequence
   */
  static PySequenceMethods tp_as_sequence;

private:
  static PyObject* tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
  static int tp_init(pyQuidditiesObject* self, PyObject* args, PyObject* kwds);
  static void tp_dealloc(pyQuidditiesObject* self);

  /**
   * @brief Gets all quiddities and transform them into pyQuiddity instances
   * @private
   * @details It uses PyCapsule types in order to encapsulate all Quiddity
   *          pointers into Python objects. This pointer is used to instanciate
   *          pyQuiddity objects.
   * @see https://docs.python.org/3/c-api/capsule.html
   * @return The list of all the quiddities of the current session.
   */
  static PyObject* get_quiddities(pyQuidditiesObject* self);

  /**
   * @brief Property getter of the `quiddities` list
   * @details It implements the __get__ method in order to apply
   *          the private `get_quiddities` method when the property
   *          is accessed.
   * @see https://docs.python.org/3/c-api/typeobj.html#c.PyTypeObject.tp_descr_get
   *      for implementation details.
   * @return The list of all the quiddities of the current session.
   */
  static PyObject* tp_descr_get(pyQuidditiesObject* self, PyObject* args, PyObject* kwds);

  /**
   * @brief Signature thats allow to use `len` method on the quiddities property.
   * @see https://docs.python.org/3/c-api/typeobj.html#c.PySequenceMethods.sq_length
   *      for implementation details.
   * @return The list of all the quiddities of the current session.
   */
  static Py_ssize_t sq_length(pyQuidditiesObject* self);

  /**
   * @brief Signature thats allow to access all the `quiddities` items with an index.
   * @see https://docs.python.org/3/c-api/typeobj.html#c.PySequenceMethods.sq_item
   *      for implementation details.
   * @return The list of all the quiddities of the current session.
   */
  static PyObject* sq_item(pyQuidditiesObject* self, Py_ssize_t i);
};

#endif
