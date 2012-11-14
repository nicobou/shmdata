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
static gboolean quiet;
static gboolean debug;
static gboolean verbose;
//static gchar **remaining_args = NULL;

static std::vector<switcher::QuiddityManager::ptr> container;

static GOptionEntry entries[] =
  {
    { "server-name", 's', 0, G_OPTION_ARG_STRING, &server_name, "server name (default is \"default\")", NULL },
    { "port-number", 'p', 0, G_OPTION_ARG_STRING, &port_number, "port number the server will bind (default is 8080)", NULL },
    { "quiet", 'q', 0, G_OPTION_ARG_NONE, &quiet, "do not display any message", NULL },
    { "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "display all messages, excluding debug", NULL },
    { "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "display all messages, including debug", NULL },
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
    if (quiet)
      return;

   switch (log_level) {
   case G_LOG_LEVEL_ERROR:
     g_print ("%s-error: %s\n",log_domain, message);
     break;
   case G_LOG_LEVEL_CRITICAL:
     g_print ("%s-critical: %s\n",log_domain, message);
     break;
   case G_LOG_LEVEL_WARNING:
     g_print ("%s-warning: %s\n",log_domain, message);
     break;
   case G_LOG_LEVEL_MESSAGE:
     if (debug || verbose)
       g_print ("%s-message: %s\n",log_domain, message);
     break;
   case G_LOG_LEVEL_INFO:
     if (debug || verbose)
       g_print ("%s-info: %s\n",log_domain, message);
     break;
   case G_LOG_LEVEL_DEBUG:
     if (debug)
       g_print ("%s-debug: %s\n",log_domain, message);
     break;
   default:
     g_print ("%s-unknown-level: %s\n",log_domain,message);
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



