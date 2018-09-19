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

#include <glib.h>
#include <locale.h>
#include <string>
#include "switcher/information-tree-json.hpp"
#include "webservices/control.nsmap"
#include "webservices/soapcontrolProxy.h"

// options
static gchar* server = nullptr;
static gboolean save = FALSE;
static gboolean load = FALSE;
static gboolean run = FALSE;
static gboolean createquiddity = FALSE;
static gboolean deletequiddity = FALSE;
static gboolean listclasses = FALSE;
static gboolean classesdoc = FALSE;
static gboolean classdoc = FALSE;
static gboolean listquiddities = FALSE;
static gboolean quidditydescr = FALSE;
static gboolean quidditiesdescr = FALSE;
static gboolean setprop = FALSE;
static gboolean getprop = FALSE;
static gboolean shmpath = FALSE;
static gboolean print_tree = FALSE;
static gboolean print_user_data = FALSE;
static gboolean prune_user_data = FALSE;
static gboolean graft_user_data = FALSE;
static gboolean tag_as_array_user_data = FALSE;
static gboolean invokemethod = FALSE;
static gchar** remaining_args = nullptr;

static GOptionEntry entries[25] = {
    {"server",
     0,
     0,
     G_OPTION_ARG_STRING,
     &server,
     "server URI (default http://localhost:27182)",
     nullptr},
    {"save", 'w', 0, G_OPTION_ARG_NONE, &save, "save history to file (--save filename)", nullptr},
    {"load",
     'x',
     0,
     G_OPTION_ARG_NONE,
     &load,
     "load state from history file (--load filename)",
     nullptr},
    // FIXME make this working { "run", nullptr, 0, G_OPTION_ARG_NONE, &run,
    // "run history to file (--run filename)", nullptr },
    {"create-quiddity",
     'C',
     0,
     G_OPTION_ARG_NONE,
     &createquiddity,
     "create a quiddity instance (-C quiddity_class [optional nick name])",
     nullptr},
    {"delete-quiddity",
     'D',
     0,
     G_OPTION_ARG_NONE,
     &deletequiddity,
     "delete a quiddity instance by its name",
     nullptr},
    {"list-classes", 'c', 0, G_OPTION_ARG_NONE, &listclasses, "list quiddity classes", nullptr},
    {"list-quiddities",
     'e',
     0,
     G_OPTION_ARG_NONE,
     &listquiddities,
     "list quiddity instances",
     nullptr},
    {"set-prop",
     's',
     0,
     G_OPTION_ARG_NONE,
     &setprop,
     "set property value (-s quiddity_name prop_name val)",
     nullptr},
    {"get-prop",
     'g',
     0,
     G_OPTION_ARG_NONE,
     &getprop,
     "get property value (-g quiddity_name prop_name)",
     nullptr},
    {"shmpath",
     'p',
     0,
     G_OPTION_ARG_NONE,
     &shmpath,
     "get shmpath of a quiddity, according to the given suffix",
     nullptr},
    {"print-tree",
     't',
     0,
     G_OPTION_ARG_NONE,
     &print_tree,
     "print information tree (-t quiddity_name [branch])",
     nullptr},
    {"print-user-data",
     'u',
     0,
     G_OPTION_ARG_NONE,
     &print_user_data,
     "print user data tree (-u quiddity_name [branch])",
     nullptr},
    {"prune-user-data",
     'r',
     0,
     G_OPTION_ARG_NONE,
     &prune_user_data,
     "prune user data tree (-r quiddity_name branch)",
     nullptr},
    {"graft-user-data",
     'a',
     0,
     G_OPTION_ARG_NONE,
     &graft_user_data,
     "graft user data tree "
     "(-a quiddity_name branch [int|float|bool|string] value)",
     nullptr},
    {"tag-as-array-user-data",
     'y',
     0,
     G_OPTION_ARG_NONE,
     &tag_as_array_user_data,
     "tag branch as array in the user data tree "
     "(-a quiddity_name branch)",
     nullptr},
    {"invoke-method",
     'i',
     0,
     G_OPTION_ARG_NONE,
     &invokemethod,
     "invoke method of a quiddity (-i quiddity_name method_name args...)",
     nullptr},
    {"classes-doc",
     'K',
     0,
     G_OPTION_ARG_NONE,
     &classesdoc,
     "print classes documentation, JSON-formated",
     nullptr},
    {"class-doc",
     'k',
     0,
     G_OPTION_ARG_NONE,
     &classdoc,
     "print class documentation, JSON-formated (--class-doc class_name)",
     nullptr},
    {"quiddities-descr",
     'Q',
     0,
     G_OPTION_ARG_NONE,
     &quidditiesdescr,
     "print instanciated quiddities, JSON-formated",
     nullptr},
    {"quiddity-descr",
     'q',
     0,
     G_OPTION_ARG_NONE,
     &quidditydescr,
     "print quiddity documentation, JSON-formated (--class-doc class_name)",
     nullptr},
    {G_OPTION_REMAINING,
     0,
     0,
     G_OPTION_ARG_STRING_ARRAY,
     &remaining_args,
     "remaining arguments",
     nullptr},
    {nullptr, 0, 0, G_OPTION_ARG_NONE, nullptr, nullptr, nullptr}};

int main(int argc, char* argv[]) {
  setlocale(LC_ALL, "");
  // command line options
  GError* error = nullptr;
  GOptionContext* context = g_option_context_new(" switcher control via webservice");
  g_option_context_add_main_entries(context, entries, nullptr);
  if (!g_option_context_parse(context, &argc, &argv, &error)) {
    g_printerr("option parsing failed: %s\n", error->message);
    exit(1);
  }

  if (server == nullptr) server = g_strdup("http://localhost:27182");

  if (!(save ^ load ^ run ^ listclasses ^ classesdoc ^ classdoc ^ listquiddities ^ quidditydescr ^
        quidditiesdescr ^ setprop ^ getprop ^ shmpath ^ createquiddity ^ deletequiddity ^
        invokemethod ^ print_tree ^ print_user_data ^ prune_user_data ^ graft_user_data ^
        tag_as_array_user_data)) {
    g_printerr(
        "I am very sorry for the inconvenience, "
        "but I am able to process only exactly one command at a time. \n");
    exit(1);
  }

  controlProxy switcher_control(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  switcher_control.soap_endpoint = server;

  if (save) {
    std::string result;
    if (remaining_args == nullptr) {
      g_printerr("file name missing\n");
      return false;
    }
    switcher_control.save(remaining_args[0], &result);
    std::cout << result << std::endl;
  } else if (load) {
    std::string result;
    if (remaining_args == nullptr) {
      g_printerr("file name missing\n");
      return false;
    }
    switcher_control.load(remaining_args[0], &result);
    std::cout << result << std::endl;
  } else if (run) {
    std::string result;
    if (remaining_args == nullptr) {
      g_printerr("file name missing\n");
      return false;
    }
    switcher_control.run(remaining_args[0], &result);
    std::cout << result << std::endl;
  } else if (listclasses) {
    std::vector<std::string> resultlist;
    switcher_control.get_classes(&resultlist);
    for (uint i = 0; i < resultlist.size(); i++) std::cout << resultlist[i] << std::endl;
  } else if (classesdoc) {
    std::string result;
    switcher_control.get_classes_doc(&result);
    std::cout << result << std::endl;
  } else if (classdoc) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("class name missing\n");
      return false;
    }
    switcher_control.get_classes_doc(&resultlist);
    std::cout << switcher::JSONSerializer::serialize(
                     switcher::JSONSerializer::deserialize(resultlist)
                         ->get_tree(std::string(".classes.") + remaining_args[0])
                         .get())
              << '\n';
  } else if (quidditydescr) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("quiddity name missing\n");
      return false;
    }
    switcher_control.get_quiddity_description(remaining_args[0], &resultlist);
    std::cout << resultlist << std::endl;
  } else if (quidditiesdescr) {
    std::string resultlist;
    switcher_control.get_quiddities_description(&resultlist);
    std::cout << resultlist << std::endl;
  } else if (listquiddities) {
    std::vector<std::string> resultlist;
    switcher_control.get_quiddity_names(&resultlist);
    for (uint i = 0; i < resultlist.size(); i++) {
      std::cout << resultlist[i] << std::endl;
    }
  } else if (print_tree) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("quiddity name missing for printing the tree\n");
      return false;
    }
    if (remaining_args[1] == nullptr)
      switcher_control.get_information_tree(remaining_args[0], ".", &resultlist);
    else
      switcher_control.get_information_tree(remaining_args[0], remaining_args[1], &resultlist);
    std::cout << resultlist << std::endl;
  } else if (print_user_data) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("quiddity name missing\n");
      return false;
    }
    if (remaining_args[1] == nullptr)
      switcher_control.get_user_data(remaining_args[0], ".", &resultlist);
    else
      switcher_control.get_user_data(remaining_args[0], remaining_args[1], &resultlist);
    std::cout << resultlist << std::endl;
  } else if (prune_user_data) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("quiddity name missing\n");
      return false;
    }
    if (remaining_args[1] == nullptr) {
      g_printerr("branch path missing\n");
      return false;
    }
    switcher_control.prune_user_data(remaining_args[0], remaining_args[1], &resultlist);
    std::cout << resultlist << std::endl;
  } else if (graft_user_data) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("quiddity name missing\n");
      return false;
    }
    if (remaining_args[1] == nullptr) {
      g_printerr("branch name missing\n");
      return false;
    }
    if (remaining_args[2] == nullptr) {
      g_printerr("type name missing\n");
      return false;
    }
    if (remaining_args[3] == nullptr) {
      g_printerr("value missing\n");
      return false;
    }
    switcher_control.graft_user_data(
        remaining_args[0], remaining_args[1], remaining_args[2], remaining_args[3], &resultlist);
    std::cout << resultlist << std::endl;
  } else if (tag_as_array_user_data) {
    std::string resultlist;
    if (remaining_args == nullptr) {
      g_printerr("quiddity name missing\n");
      return false;
    }
    if (remaining_args[1] == nullptr) {
      g_printerr("branch name missing\n");
      return false;
    }
    if (remaining_args[2] == nullptr) {
      g_printerr("value missing\n");
      return false;
    }
    auto val = std::string(remaining_args[2]);
    if (val != "true" && val != "false") {
      g_printerr("value must be true or false\n");
      return false;
    }
    switcher_control.tag_as_array_user_data(
        remaining_args[0], remaining_args[1], val == "true" ? true : false, &resultlist);
    std::cout << resultlist << std::endl;
  } else if (setprop) {
    if (remaining_args == nullptr || remaining_args[1] == nullptr || remaining_args[2] == nullptr) {
      g_printerr("missing argument for set property\n");
      return false;
    }
    // special since on
    switcher_control.send_set_property(remaining_args[0], remaining_args[1], remaining_args[2]);
    if (switcher_control.recv_set_property_empty_response())
      switcher_control.soap_print_fault(stderr);
    // connection should not be kept alive after the last call: be nice to the
    // server and tell it that we close the connection after this call
    soap_clr_omode(&switcher_control, SOAP_IO_KEEPALIVE);
    switcher_control.soap_close_socket();
    return 0;
  } else if (getprop) {
    if (remaining_args == nullptr || remaining_args[1] == nullptr) {
      g_printerr("missing argument for get property\n");
      return false;
    }
    std::string val;
    switcher_control.get_property(remaining_args[0], remaining_args[1], &val);
    std::cout << val << std::endl;
  } else if (shmpath) {
    if (remaining_args == nullptr || remaining_args[1] == nullptr) {
      g_printerr("missing argument for shmpath\n");
      return false;
    }
    std::string val;
    switcher_control.make_shmpath(remaining_args[0], remaining_args[1], &val);
    std::cout << val << std::endl;
  } else if (createquiddity) {
    if (remaining_args == nullptr) {
      g_printerr("missing class name for creating quiddity\n");
      return false;
    }
    std::string name;
    if (remaining_args[1] == nullptr)
      switcher_control.create_named_quiddity(remaining_args[0], std::string(), &name);
    else
      switcher_control.create_named_quiddity(remaining_args[0], remaining_args[1], &name);
    std::cout << name << std::endl;
  } else if (deletequiddity) {
    if (remaining_args == nullptr) {
      g_printerr("missing quiddity name for deleting quiddity\n");
      return false;
    }
    switcher_control.delete_quiddity(remaining_args[0]);
  } else if (invokemethod) {
    if (remaining_args == nullptr || remaining_args[1] == nullptr) {
      g_printerr("not enough argument for invoking a function\n");
      return false;
    }
    std::vector<std::string> args;
    int i = 2;
    while (remaining_args[i] != nullptr) {
      args.push_back(remaining_args[i]);
      i++;
    }

    std::string result;
    switcher_control.invoke_method(remaining_args[0], remaining_args[1], args, &result);
    g_print("%s\n", result.c_str());
  }

  if (switcher_control.error) switcher_control.soap_stream_fault(std::cerr);

  return 0;
}
