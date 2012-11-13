/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "switcher/quiddity-manager.h"
#include <iostream>
#include <signal.h>


static gchar *server_name = NULL;
static gchar *port_number = NULL;
//static gchar **remaining_args = NULL;
static std::vector<switcher::QuiddityManager::ptr> container;

static GOptionEntry entries[] =
  {
    { "server-name", 's', 0, G_OPTION_ARG_STRING, &server_name, "server name (default is \"default\")", NULL },
    { "port-number", 'p', 0, G_OPTION_ARG_STRING, &port_number, "port number the server will bind (default is 8080)", NULL },
    { NULL }
  };

void
leave (int sig)
{
  //removing reference to manager in order to delete it
  container.clear ();
  exit (sig);
}

 static void
 log_handler (const gchar *log_domain, 
 	     GLogLevelFlags log_level,
 	     const gchar *message,
 	     gpointer user_data)
 {
   //OsgReader_impl *context = static_cast<OsgReader_impl*>(user_data);
   switch (log_level) {
   case G_LOG_LEVEL_ERROR:
     g_print ("%s, ERROR: %s\n",G_LOG_DOMAIN, message);
     break;
   case G_LOG_LEVEL_CRITICAL:
     g_print ("%s, CRITICAL: %s\n",G_LOG_DOMAIN, message);
     break;
   case G_LOG_LEVEL_WARNING:
     g_print ("%s, WARNING: %s\n",G_LOG_DOMAIN, message);
     break;
   case G_LOG_LEVEL_MESSAGE:
     g_print ("%s, MESSAGE: %s\n",G_LOG_DOMAIN, message);
     break;
   case G_LOG_LEVEL_INFO:
     g_print ("%s, INFO: %s\n",G_LOG_DOMAIN, message);
     break;
   case G_LOG_LEVEL_DEBUG:
     g_print ("%s, DEBUG: %s\n",G_LOG_DOMAIN, message);
     break;
   default:
     g_print ("%s: %s\n",G_LOG_DOMAIN,message);
     break;
   }
 }

int
main (int argc,
      char *argv[])
{
  (void) signal (SIGINT, leave);

  //command line options
  GError *error = NULL;
  GOptionContext *context;
  context = g_option_context_new ("- switcher server");
  g_option_context_add_main_entries (context, entries, NULL);
  if (!g_option_context_parse (context, &argc, &argv, &error))
    {
      g_print ("option parsing failed: %s\n", error->message);
      exit (1);
    } 

  if (server_name == NULL)
    server_name = "default";
  if (port_number == NULL)
    port_number = "8080";

  g_log_set_default_handler (log_handler, NULL);
  
  {//using context in order to let excluse ownership of manager by the container
    switcher::QuiddityManager::ptr manager 
      = switcher::QuiddityManager::make_manager (server_name);  
    container.push_back (manager); // keep reference only in the container
    // Create a runtime (pipeline0)
    //std::string runtime = 
    manager->create ("runtime");
  
    std::string soap_name = manager->create ("SOAPcontrolServer", "soapserver");
    std::vector<std::string> port_arg;
    port_arg.push_back (port_number);
    manager->invoke (soap_name, "set_port", port_arg);
  }

  //waiting for end of life
  while (1)
    sleep (1);
  
  return 0;
}



