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

#ifndef __SWITCHER_PYQUIDDITY_H__
#define __SWITCHER_PYQUIDDITY_H__

#include <Python.h>  // according to python doc, this *must* be the first include

#include <future>
#include <list>
#include <map>
#include <memory>

#include "switcher/quiddity/quiddity.hpp"
#include "switcher/switcher.hpp"

using namespace switcher;
using namespace quiddity;
using namespace claw;

class pyQuiddity {
 public:
  using sig_registering_t = struct {
    std::map<signal::sig_id_t, signal::register_id_t> signals{};
    std::map<signal::sig_id_t, PyObject*> callbacks{};
    std::map<signal::sig_id_t, PyObject*> user_data{};
  };
  using prop_registering_t = struct {
    std::map<property::prop_id_t, property::register_id_t> props{};
    std::map<property::prop_id_t, PyObject*> callbacks{};
    std::map<property::prop_id_t, PyObject*> user_data{};
  };
  using pyQuiddityObject = struct {
    PyObject_HEAD std::weak_ptr<Quiddity> quid{};
    PyObject* switcher{nullptr};
    PyObject* props{nullptr};
    PyObject* signals{nullptr};
    InfoTree::ptr connnection_spec_keep_alive_{};
    std::unique_ptr<sig_registering_t> sig_reg{};
    std::unique_ptr<prop_registering_t> prop_reg{};
    // async invocations
    std::unique_ptr<std::list<std::future<void>>> async_invocations{};
    // the default thread state used in order to generate unique thread
    // states when c++ callbacks race against the GIL. Thanks ! to Sam:
    // https://www.codevate.com/blog/7-concurrency-with-embedded-python-in-a-multi-threaded-c-application
    PyInterpreterState* interpreter_state{nullptr};
  };

  static PyTypeObject pyType;
  static PyMethodDef pyQuiddity_methods[];
  static PyGetSetDef tp_getset[];

 private:
  // Boilerplate
  static PyObject* tp_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/);

  /**
   * @brief Main method that initializes the pyQuiddity object
   * @param self The instanciated pyQuiddity object
   * @param args The instance arguments when instanciated
   * @param kwds The instance arguments when instanciated
   * @details This method is **encapsulating a created quiddity** and
   *          can't be used without a reference on an existing quiddity.
   *          In order to create new quiddities from the pyquid API,
   *          we recommand to use the `pyswitch.create(*args)` method.
   * @return 1 if the initialization failed, 0 if it succeed
   */
  static int Quiddity_init(pyQuiddityObject* self, PyObject* /*args*/, PyObject* /*kwds*/);
  static void Quiddity_dealloc(pyQuiddityObject* self);
  static PyObject* tp_repr(pyQuiddityObject* self);
  static PyObject* tp_str(pyQuiddityObject* self);
  static PyObject* set(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* invoke(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* invoke_async(pyQuiddityObject* self, PyObject* args, PyObject* kwds);

  static PyObject* notify_update(pyQuiddityObject* self, PyObject* args, PyObject* kwds);

  /**
   * @brief Get the properties registry of the pyQuiddity object
   * @details The properties registry is a pythonic interface to
   *          the `properties` entry in the quiddity's InfoTree.
   *          It instanciates a descriptor and a registry from
   *          the `props.py` file and it is initialized under
   *          the `props` attribute of pyQuiddity objects.
   *
   *          So, it allows you to discover properties like this:
   *          ```py
   *          vid = sw.create('videotestsrc', 'ex')
   *          print(vid.props)
   *          assert vid.props['started'] == False
   *          vid.props['started'] = True
   *          ```
   *
   *          See https://docs.python.org/3/howto/descriptor.html
   *          for implementation details.
   *
   * @param self The pyQuiddity instance to initialize
   * @return The descriptor object
   */
  static PyObject* get_property_descriptor(pyQuiddityObject* self);

  /**
   * @brief Get the signals registry of the pyQuiddity object
   * @param self The pyQuiddity instance to initialize
   * @return The descriptor object
   */
  static PyObject* get_signal_descriptor(pyQuiddityObject* self);

  // access to user tree
  static PyObject* get_user_tree(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* notify_user_data_grafted(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* notify_user_data_pruned(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // access to quiddity InfoTree
  static PyObject* get_info(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_info_tree_as_json(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // kind, nickname and id
  static PyObject* get_kind(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* set_nickname(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* nickname(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* id(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // subscription
  static PyObject* subscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* unsubscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static bool subscribe_to_signal(pyQuiddityObject* self,
                                  const char* signal_name,
                                  PyObject* cb,
                                  PyObject* user_data);
  static bool unsubscribe_from_signal(pyQuiddityObject* self, const char* signal_name);
  static bool subscribe_to_property(pyQuiddityObject* self,
                                    const char* prop_name,
                                    PyObject* cb,
                                    PyObject* user_data);
  static bool unsubscribe_from_property(pyQuiddityObject* self, const char* prop_name);
  // signals
  static PyObject* get_signal_id(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // claw
  static PyObject* try_connect(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_connection_specs(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_writer_claws(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_follower_claws(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
};

#endif
