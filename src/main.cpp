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

#include <locale.h>
#include <signal.h>
#include <time.h>
#include <vector>
#include "switcher/console-logger.hpp"
#include "switcher/file-utils.hpp"
#include "switcher/information-tree-json.hpp"
#include "switcher/quiddity-class-printer.hpp"
#include "switcher/silent-logger.hpp"
#include "switcher/switcher.hpp"

using namespace switcher;

static const gchar* server_name = nullptr;
static const gchar* port_number = nullptr;
static const gchar* load_file = nullptr;
static gboolean display_version;
static gboolean quiet;
static gboolean debug;
static gboolean verbose;
static gboolean listclasses;
static gboolean classesdoc;
static gchar* classdoc = nullptr;
static gchar* extraplugindir = nullptr;
static Switcher::ptr manager;
static GOptionEntry entries[15] = {
    {"version",
     'V',
     0,
     G_OPTION_ARG_NONE,
     &display_version,
     "display switcher version number",
     nullptr},
    {"server-name",
     'n',
     0,
     G_OPTION_ARG_STRING,
     &server_name,
     "server name (default is \"default\")",
     nullptr},
    {"port-number",
     'p',
     0,
     G_OPTION_ARG_STRING,
     &port_number,
     "port number the server will bind (default is 27182)",
     nullptr},
    {"load",
     'l',
     0,
     G_OPTION_ARG_STRING,
     &load_file,
     "load state from history file (-l filename)",
     nullptr},
    {"quiet", 'q', 0, G_OPTION_ARG_NONE, &quiet, "do not display any message", nullptr},
    {"verbose",
     'v',
     0,
     G_OPTION_ARG_NONE,
     &verbose,
     "display all messages, excluding debug",
     nullptr},
    {"debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "display all messages, including debug", nullptr},
    {"list-classes", 'C', 0, G_OPTION_ARG_NONE, &listclasses, "list quiddity classes", nullptr},
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
     G_OPTION_ARG_STRING,
     &classdoc,
     "print class documentation, JSON-formated (--class-doc class_name)",
     nullptr},
    {"extra-plugin-dir",
     'E',
     0,
     G_OPTION_ARG_STRING,
     &extraplugindir,
     "directory where to find additional plugins",
     nullptr}};

void leave(int sig) {
  // removing reference to manager in order to delete it
  {
    Switcher::ptr empty;
    manager.swap(empty);
  }
  exit(sig);
}

int main(int argc, char* argv[]) {
  setlocale(LC_ALL, "");
  (void)signal(SIGINT, leave);
  (void)signal(SIGABRT, leave);
  (void)signal(SIGQUIT, leave);
  (void)signal(SIGTERM, leave);

  // command line options
  GError* error = nullptr;
  GOptionContext* context;
  context = g_option_context_new("- switcher server");
  g_option_context_add_main_entries(context, entries, nullptr);
  if (!g_option_context_parse(context, &argc, &argv, &error)) {
    g_printerr("option parsing failed: %s\n", error->message);
    exit(1);
  }

  // running a switcher server
  if (server_name == nullptr) server_name = "default";
  if (port_number == nullptr) port_number = "27182";

  if (debug) {
    manager = Switcher::make_switcher<ConsoleLogger>(server_name);
  } else {
    manager = Switcher::make_switcher<SilentLogger>(server_name);
  }

  if (display_version) {
    std::cout << manager->get_switcher_version() << '\n';
    return 0;
  }

  // loading plugins from default location // FIXME add an option
  manager->factory<MPtr(&quid::Factory::scan_dir)>(
      manager->factory<MPtr(&quid::Factory::get_default_plugin_dir)>());

  if (extraplugindir != nullptr) manager->factory<MPtr(&quid::Factory::scan_dir)>(extraplugindir);

  // checking if this is printing info only
  if (listclasses) {
    std::vector<std::string> resultlist = manager->factory<MPtr(&quid::Factory::get_class_list)>();
    for (uint i = 0; i < resultlist.size(); i++) g_print("%s\n", resultlist[i].c_str());
    return 0;
  }
  if (classesdoc) {
    g_print(
        "%s\n",
        QuiddityClassPrinter::print(manager->factory<MPtr(&quid::Factory::get_classes_doc)>().get())
            .c_str());
    return 0;
  }
  if (classdoc != nullptr) {
    g_print("%s\n",
            JSONSerializer::serialize(manager->factory<MPtr(&quid::Factory::get_classes_doc)>()
                                          ->get_tree(std::string(".classes.") + classdoc)
                                          .get())
                .c_str());
    return 0;
  }

  auto created = manager->quids<MPtr(&quid::Container::create)>("SOAPcontrolServer", "soapserver");
  if (!created) {
    std::cerr << "could not create SOAP server" << '\n';
    return 0;
  }
  auto soap = created.get();
  if (!soap->meth<MPtr(&MContainer::invoke_str)>(soap->meth<MPtr(&MContainer::get_id)>("set_port"),
                                                 serialize::esc_for_tuple(port_number))) {
    std::cerr << "could not set soap port " << '\n';
    return 0;
  }

  manager->reset_state(false);

  if (load_file &&
      !manager->load_state(JSONSerializer::deserialize(FileUtils::get_content(load_file)).get())) {
    std::cerr << "could not load file " << load_file << '\n';
  }

  // waiting for end of life
  timespec delay;
  delay.tv_sec = 1;
  delay.tv_nsec = 0;
  while (1) nanosleep(&delay, nullptr);

  return 0;
}
