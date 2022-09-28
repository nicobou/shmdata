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

#ifndef __SWITCHER_PYSWITCH_H__
#define __SWITCHER_PYSWITCH_H__

#include <Python.h>  // according to python doc, this *must* be the first include
#include <structmember.h>

#include <map>

#include "./pybundle.hpp"
#include "./pylogger.hpp"
#include "./pysession.hpp"
#include "switcher/session/session.hpp"
#include "switcher/switcher.hpp"

using namespace switcher;

PyObject* InterpType(const char* type_name, const char* module_name);

class pySwitch {
 public:
  using sig_registering_t = struct {
    std::map<std::string, unsigned int> signals{};
    std::map<std::string, PyObject*> callbacks{};
    std::map<std::string, PyObject*> user_data{};
  };
  using pySwitchObject = struct {
    PyObject_HEAD PyObject* name{nullptr};
    Switcher::ptr switcher{};
    PyObject* bundles{nullptr};
    PyObject* logger{nullptr};
    PyObject* quiddities{nullptr};
    PyObject* session{nullptr};
    std::unique_ptr<sig_registering_t> sig_reg{};
    PyInterpreterState* interpreter_state{nullptr};
  };

  static PyMemberDef pySwitch_members[];
  static PyMethodDef pySwitch_methods[];

  // type
  static PyTypeObject pyType;

 private:
  static void Switcher_dealloc(pySwitchObject* self);
  static PyObject* Switcher_new(PyTypeObject* type, PyObject* args, PyObject* kwds);

  /**
   * @brief Main method that initializes the pySwitch object
   * @param self The instanciated pySwitch object
   * @param args The instance arguments when instanciated
   * @param kwds The instance arguments when instanciated
   * @details This method is wrapping a switcher instance
   *          and provides all higher APIs for:
   *            - quiddity creation, removal and query
   *            - signal subscriptions and unsubscription
   * @return 1 if the initialization failed, 0 if it succeed
   */
  static int Switcher_init(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* name(pySwitchObject* self);
  static PyObject* version(pySwitchObject* self);

  // descriptors init
  /**
   * @brief Get the logger descriptor of the pySwitch object
   * @param self The pySwitch instance to initialize
   * @return The descriptor object
   */
  static PyObject* get_logger_descriptor(pySwitchObject* self);

  /**
   * @brief Get the session registry of the pySwitch object
   * @param self The pySwitch instance to initialize
   * @details This descriptor is attached to the `session` attribute
   *          of the pySwitch instance. So, using this descriptor is
   *          pretty much like doing the following in Python:
   *
   *          ```py
   *          class Session:
   *            def __init__(self, instance):
   *            self.instance = instance;
   *
   *          class Switcher:
   *            def __init__(self):
   *            self.session = Session(self);
   *          ```
   *
   *          Anytime a Switcher instance is initialized,
   *          another Session instance is also created.
   *          For convenience, both instances might keep a reference
   *          to each other.
   *
   * @return The descriptor object
   */
  static PyObject* get_session_descriptor(pySwitchObject* self);

  /**
   * @brief Get the bundles registry of the pySwitch object
   * @param self The pySwitch instance to initialize
   * @return The descriptor object
   */
  static PyObject* get_bundles_descriptor(pySwitchObject* self);

  // quiddity life
  static PyObject* create(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* remove(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_quid(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_quid_id(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* load_bundles(pySwitchObject* self, PyObject* args, PyObject* kwds);
  // state saving
  static PyObject* get_state(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* load_state(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* reset_state(pySwitchObject* self, PyObject* args, PyObject* kwds);
  // introspection
  static PyObject* list_kinds(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* kinds_doc(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* kind_doc(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* list_quids(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* list_ids(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* quids_descr(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* quid_descr(pySwitchObject* self, PyObject* args, PyObject* kwds);
  // signals
  static bool subscribe_to_signal(pySwitchObject* self,
                                  const std::string signal_name,
                                  PyObject* cb,
                                  PyObject* user_data);
  static PyObject* subscribe(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static bool unsubscribe_from_signal(pySwitchObject* self, const std::string signal_name);
  static PyObject* unsubscribe(pySwitchObject* self, PyObject* args, PyObject* kwds);

  static PyObject* list_extra_configs(pySwitchObject* self, PyObject* args, PyObject* kwds);
  static PyObject* read_extra_config(pySwitchObject* self, PyObject* args, PyObject* kwds);
};

#endif
