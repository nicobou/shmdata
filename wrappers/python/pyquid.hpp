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

#ifndef __SWITCHER_PYQUID_H__
#define __SWITCHER_PYQUID_H__

#include <Python.h>  // according to python doc, this *must* be the first include

#ifdef __cplusplus
extern "C" {
#endif

static PyMethodDef pyquid_QuidMethods[] = {
    {nullptr, nullptr, 0, nullptr} /* Sentinel */
};

PyDoc_STRVAR(pyquid_quiddity_doc,
             "PyQuid is a python module for libswitcher.\n"
             "A Switcher is a manager of several instances of Quiddities. "
             "Each Quiddity can be created and removed dynamically from a Switcher instance."
             "Quiddities can be controlled through property get/set and method invocations.\n"
             "Most Quiddities are exposing and/or consuming live stream of data frames using the "
             "shmdata library.\n"
             "\n"
             "For instance, the camera (v4l2src) Quiddity provides the video stream from a camera, "
             "that can be connected simultaneously to several other Quidditipes, including a local "
             "display (glfwin) a file recorder (avrec), a streamer (rtmp) or an other shmdata "
             "enabled software. "
             "\n"
             "(Note that shmdata makes data streams shareable with all other Quiddities, "
             "but also with other processes.\n");

static struct PyModuleDef pyquid_quidmodule = {
    PyModuleDef_HEAD_INIT,
    "quid",              /* name of module */
    pyquid_quiddity_doc, /* module documentation, may be nullptr */
    -1,                  /* size of per-interpreter state of the module,
                            or -1 if the module keeps state in global variables. */
    pyquid_QuidMethods};

PyMODINIT_FUNC PyInit_pyquid(void);

#ifdef __cplusplus
}
#endif

#endif
