/*
 * This file is part of switcher python wrapper.
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

#include "./pyquid.hpp"
#include "./pyinfotree.hpp"
#include "./pyqrox.hpp"
#include "./pyquiddity.hpp"
#include "./pyswitch.hpp"

PyMODINIT_FUNC PyInit_pyquid(void) {
  PyObject* m;

  if (PyType_Ready(&pySwitch::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyQrox::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyQuiddity::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyInfoTree::pyType) < 0) return nullptr;

  m = PyModule_Create(&pyquid_quidmodule);
  if (m == nullptr) return nullptr;

  Py_INCREF(&pySwitch::pyType);
  PyModule_AddObject(m, "Switcher", reinterpret_cast<PyObject*>(&pySwitch::pyType));
  Py_INCREF(&pyQrox::pyType);
  PyModule_AddObject(m, "Qrox", reinterpret_cast<PyObject*>(&pyQrox::pyType));
  Py_INCREF(&pyQuiddity::pyType);
  PyModule_AddObject(m, "Quiddity", reinterpret_cast<PyObject*>(&pyQuiddity::pyType));
  Py_INCREF(&pyInfoTree::pyType);
  PyModule_AddObject(m, "InfoTree", reinterpret_cast<PyObject*>(&pyInfoTree::pyType));

  return m;
}
