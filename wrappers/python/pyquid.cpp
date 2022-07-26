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

#include "./pyfclaw.hpp"
#include "./pyinfotree.hpp"
#include "./pyquiddity.hpp"
#include "./pysession.hpp"
#include "./pyswitch.hpp"
#include "./pywclaw.hpp"

PyMODINIT_FUNC PyInit_pyquid(void) {
  PyObject* m;

  if (PyType_Ready(&pySwitch::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyQuiddity::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyInfoTree::pyType) < 0) return nullptr;
  if (PyType_Ready(&BundleManager::pyType) < 0) return nullptr;
  if (PyType_Ready(&pySession::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyLogger::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyWriterClaw::pyType) < 0) return nullptr;
  if (PyType_Ready(&pyFollowerClaw::pyType) < 0) return nullptr;

  m = PyModule_Create(&pyquid_quidmodule);
  if (m == nullptr) return nullptr;

  PyObject* BundleType = InterpType("Bundle", "bundles.py");
  Py_INCREF(BundleType);
  PyModule_AddObject(m, "Bundle", BundleType);
  Py_INCREF(&pySwitch::pyType);
  PyModule_AddObject(m, "Switcher", reinterpret_cast<PyObject*>(&pySwitch::pyType));
  Py_INCREF(&pyQuiddity::pyType);
  PyModule_AddObject(m, "Quiddity", reinterpret_cast<PyObject*>(&pyQuiddity::pyType));
  Py_INCREF(&pyInfoTree::pyType);
  PyModule_AddObject(m, "InfoTree", reinterpret_cast<PyObject*>(&pyInfoTree::pyType));
  Py_INCREF(&pyWriterClaw::pyType);
  PyModule_AddObject(m, "WriterClaw", reinterpret_cast<PyObject*>(&pyWriterClaw::pyType));
  Py_INCREF(&pyFollowerClaw::pyType);
  PyModule_AddObject(m, "FollowerClaw", reinterpret_cast<PyObject*>(&pyFollowerClaw::pyType));

  return m;
}
