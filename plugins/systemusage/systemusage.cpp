/*
 * This file is part of switcher-top.
 *
 * switcher-top is free software; you can redistribute it and/or
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

#include "systemusage.h"
#include "switcher/information-tree-basic-serializer.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#define PROCSTATFILE "/proc/stat"
#define PROCMEMINFOFILE "/proc/meminfo"

using namespace std;
using namespace switcher::data;

namespace switcher
{

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SystemUsage,
				       "SystemUsage plugin",
				       "SystemUsage", 
				       "Gives system load information",
				       "LGPL",
				       "systemusage",				
				       "Emmanuel Durand");
  SystemUsage::SystemUsage () :
    cpuNbr_(0),
    refreshRate_(2)
  {
  }
  
  SystemUsage::~SystemUsage ()
  {
    running_ = false;
    pollStateThread_->join ();
  }

  bool
  SystemUsage::init ()
  {
    // Initialize the properties tree
    tree_ = make_tree ();

    // Launch the polling thread
    running_ = true;
    pollStateThread_ = make_shared<thread>([&] () {
      pollState();
    });

    // Trying to reach the /proc files
    ifstream file;
    file.open(PROCSTATFILE);
    if (!file.is_open())
      return false;

    for (string line; getline(file, line);)
    {
      string substr = line.substr(0, 3);
      if (substr == "cpu")
        cpuNbr_++;
    }
    cpuNbr_ = cpuNbr_ == 1 ? 1 : cpuNbr_ - 1;
    file.close();

    file.open(PROCMEMINFOFILE);
    if (!file.is_open())
      return false;
    file.close();

    return true;
  }
  
  void
  SystemUsage::pollState ()
  {
    bool firstRun = true;

    while (running_)
    {
      ifstream file;

      // Get the cpu state
      file.open(PROCSTATFILE);
      if (!file.is_open())
        return;

      for (string line; getline(file, line);)
      {
        string core;
        int user, nice, system, idle, io, irq, softIrq, steal, guest;

        istringstream stream(line);
        stream >> core >> user >> nice >> system >> idle >> io >> irq >> softIrq >> steal >> guest;

        if (core.substr(0, 3) == "cpu")
        {
          int totalTime = user + nice + system + idle + io + irq + softIrq + steal + guest;
          float totalP, userP, niceP, systemP, idleP;
          totalP = (float)(user + nice + system) / (float)totalTime;
          userP = (float)user / (float)totalTime;
          niceP = (float)nice / (float)totalTime;
          systemP = (float)system / (float)totalTime;
          idleP = (float)idle / (float)totalTime;

          // Initialize the tree
          if (firstRun)
          {
            tree_->graft(".cpu." + core + ".total", make_tree());
            tree_->graft(".cpu." + core + ".user", make_tree());
            tree_->graft(".cpu." + core + ".nice", make_tree());
            tree_->graft(".cpu." + core + ".system", make_tree());
            tree_->graft(".cpu." + core + ".idle", make_tree());
          }

          tree_->set_data(".cpu." + core + ".total", totalP);
          tree_->set_data(".cpu." + core + ".user", userP);
          tree_->set_data(".cpu." + core + ".nice", niceP);
          tree_->set_data(".cpu." + core + ".system", systemP);
          tree_->set_data(".cpu." + core + ".idle", idleP);
        }
      }
      file.close();

      // Get the memory state
      file.open(PROCMEMINFOFILE);
      if (!file.is_open())
        return;

      for (string line; getline(file, line);)
      {
        // Initialize the tree
        if (firstRun)
        {
          tree_->graft(".mem.total", make_tree());
          tree_->graft(".mem.free", make_tree());
          tree_->graft(".mem.buffers", make_tree());
          tree_->graft(".mem.cached", make_tree());
          tree_->graft(".mem.swap_total", make_tree());
          tree_->graft(".mem.swap_free", make_tree());
        }

        string type;
        int qty;

        istringstream stream(line);
        stream >> type >> qty;

        if (type.find("MemTotal") != string::npos)
          tree_->set_data("mem.total", qty);
        else if (type.find("MemFree") != string::npos)
          tree_->set_data("mem.free", qty);
        else if (type.find("Buffers") != string::npos)
          tree_->set_data("mem.buffers", qty);
        else if (type.find("Cached") != string::npos)
          tree_->set_data("mem.cached", qty);
        else if (type.find("SwapTotal") != string::npos)
          tree_->set_data("mem.swap_total", qty);
        else if (type.find("SwapFree") != string::npos)
          tree_->set_data("mem.swap_free", qty);
      }

      graft_tree (".top.", tree_);

      firstRun = false;

      this_thread::sleep_for(chrono::milliseconds(1000 / refreshRate_));
    }
  }
}
