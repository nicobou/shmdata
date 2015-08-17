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

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "switcher/information-tree-basic-serializer.hpp"
#include "switcher/std2.hpp"
#include "./systemusage.hpp"


#define PROCSTATFILE "/proc/stat"
#define PROCMEMINFOFILE "/proc/meminfo"
#define PROCNETDEVFILE "/proc/net/dev"

using namespace std;
using namespace
switcher::data;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    SystemUsage,
    "systemusage",
    "SystemUsage plugin",
    "monitoring",
    "",
    "Gives system load information",
    "LGPL",
    "Emmanuel Durand");

SystemUsage::SystemUsage(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper> ()),
    tree_{Tree::make()},
    period_(1.0),
    pollStateTask_(std2::make_unique<PeriodicTask>([this](){
          this->pollState();
        }, std::chrono::milliseconds(static_cast<int>(1000 * period_)))){
}

bool SystemUsage::init() {
  period_prop_ = custom_props_->make_double_property("period",  // name
                                                     "Update period",  // description
                                                     0.1,
                                                     5.0,
                                                     period_, (GParamFlags)
                                                     G_PARAM_READWRITE,
                                                     SystemUsage::setRefreshPeriod,
                                                     SystemUsage::getRefreshPeriod,
                                                     this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            period_prop_,
                            "period",
                            "Update period");   // long name
  return init_tree();
}

bool SystemUsage::init_tree(){
  // Trying to reach the /proc files
  ifstream file;
  // init cpu
  file.open(PROCSTATFILE);
  if (!file.is_open())
    return false;
  for (string line; getline(file, line);) {
    string core;
    long user, nice, system, idle, io, irq, softIrq, steal, guest;
    istringstream stream(line);
    stream >> core >> user >> nice >> system >> idle >> io >> irq >>
          softIrq >> steal >> guest;
    if (core.substr(0, 3) == "cpu") {
      tree_->graft(".cpu." + core + ".total", Tree::make());
      tree_->graft(".cpu." + core + ".user", Tree::make());
      tree_->graft(".cpu." + core + ".nice", Tree::make());
      tree_->graft(".cpu." + core + ".system", Tree::make());
      tree_->graft(".cpu." + core + ".idle", Tree::make());
    }
  }
  file.close();
  // init mem
  file.open(PROCMEMINFOFILE);
  if (!file.is_open())
    return false;
  for (string line; getline(file, line);) {
        tree_->graft(".mem.total", Tree::make());
        tree_->graft(".mem.free", Tree::make());
        tree_->graft(".mem.buffers", Tree::make());
        tree_->graft(".mem.cached", Tree::make());
        tree_->graft(".mem.swap_total", Tree::make());
        tree_->graft(".mem.swap_free", Tree::make());
  }
  file.close();
  // init net
  file.open(PROCNETDEVFILE);
  if (!file.is_open())
    return false;
  for (string line; getline(file, line);) {
    string netI;
    long rBytes, rPackets, rErrs, rDrop, rFifo, rFrame, rCompressed, rMulticast;
    long tBytes, tPackets, tErrs, tDrop, tFifo, tColls, tCarrier, tCompressed;
    istringstream stream(line);
    stream >> netI >> rBytes >> rPackets >> rErrs >> rDrop >> rFifo >>
        rFrame >> rCompressed >> rMulticast >> tBytes >> tPackets >>
        tErrs >> tDrop >> tFifo >> tColls >> tCarrier >> tCompressed;
    if (netI.find("Inter") == string::npos
        && netI.find("face") == string::npos) {
      string netName;
      netName = netI.substr(0, netI.find(":"));
      tree_->graft(".net." + netName + ".rx_rate", Tree::make());
      tree_->graft(".net." + netName + ".rx_bytes", Tree::make());
      tree_->graft(".net." + netName + ".rx_packets", Tree::make());
      tree_->graft(".net." + netName + ".rx_errors", Tree::make());
      tree_->graft(".net." + netName + ".rx_drop", Tree::make());
      tree_->graft(".net." + netName + ".tx_rate", Tree::make());
      tree_->graft(".net." + netName + ".tx_bytes", Tree::make());
      tree_->graft(".net." + netName + ".tx_packets", Tree::make());
      tree_->graft(".net." + netName + ".tx_errors", Tree::make());
      tree_->graft(".net." + netName + ".tx_drop", Tree::make());
      _net[netName] = Net();
    }
  }
  file.close();
  return true;
}

void
SystemUsage::pollState() {
    ifstream file;
    // Get the cpu state
    file.open(PROCSTATFILE);
    if (!file.is_open())
      return;
    for (string line; getline(file, line);) {
      string core;
      long user, nice, system, idle, io, irq, softIrq, steal, guest;
      istringstream stream(line);
      stream >> core >> user >> nice >> system >> idle >> io >> irq >>
          softIrq >> steal >> guest;
      if (core.substr(0, 3) == "cpu") {
        long totalTime = user + nice + system + idle + io + irq + softIrq + steal + guest;
        if (totalTime == _cpus[core].totalTime)
          continue;
        float totalP, userP, niceP, systemP, idleP;
        userP = (float) (user - _cpus[core].user) / (float) (totalTime - _cpus[core].totalTime);
        niceP = (float) (nice - _cpus[core].nice) / (float) (totalTime - _cpus[core].totalTime);
        systemP =
            (float) (system - _cpus[core].system) / (float) (totalTime - _cpus[core].totalTime);
        idleP = (float) (idle - _cpus[core].idle) / (float) (totalTime - _cpus[core].totalTime);
        totalP = userP + niceP + systemP;
        tree_->set_data(".cpu." + core + ".total", totalP);
        tree_->set_data(".cpu." + core + ".user", userP);
        tree_->set_data(".cpu." + core + ".nice", niceP);
        tree_->set_data(".cpu." + core + ".system", systemP);
        tree_->set_data(".cpu." + core + ".idle", idleP);
        _cpus[core].user = user;
        _cpus[core].nice = nice;
        _cpus[core].system = system;
        _cpus[core].idle = idle;
        _cpus[core].io = io;
        _cpus[core].irq = irq;
        _cpus[core].softIrq = softIrq;
        _cpus[core].steal = steal;
        _cpus[core].guest = guest;
        _cpus[core].totalTime = totalTime;
      }
    }
    file.close();

    // Get the memory state
    file.open(PROCMEMINFOFILE);
    if (!file.is_open())
      return;
    for (string line; getline(file, line);) {
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
      else if (type.find("Cached") != string::npos
               && type.find("SwapCached") == string::npos)
        tree_->set_data("mem.cached", qty);
      else if (type.find("SwapTotal") != string::npos)
        tree_->set_data("mem.swap_total", qty);
      else if (type.find("SwapFree") != string::npos)
        tree_->set_data("mem.swap_free", qty);
    }
    file.close();

    // Get the network state
    file.open(PROCNETDEVFILE);
    if (!file.is_open())
      return;
    for (string line; getline(file, line);) {
      string netI;
      long rBytes, rPackets, rErrs, rDrop, rFifo, rFrame, rCompressed, rMulticast;
      long tBytes, tPackets, tErrs, tDrop, tFifo, tColls, tCarrier, tCompressed;
      istringstream stream(line);
      stream >> netI >> rBytes >> rPackets >> rErrs >> rDrop >> rFifo >>
          rFrame >> rCompressed >> rMulticast >> tBytes >> tPackets >>
          tErrs >> tDrop >> tFifo >> tColls >> tCarrier >> tCompressed;
      if (netI.find("Inter") == string::npos
          && netI.find("face") == string::npos) {
        string netName;
        netName = netI.substr(0, netI.find(":"));
        if (false) {  //FIXME
          tree_->graft(".net." + netName + ".rx_rate", Tree::make());
          tree_->graft(".net." + netName + ".rx_bytes", Tree::make());
          tree_->graft(".net." + netName + ".rx_packets", Tree::make());
          tree_->graft(".net." + netName + ".rx_errors", Tree::make());
          tree_->graft(".net." + netName + ".rx_drop", Tree::make());
          tree_->graft(".net." + netName + ".tx_rate", Tree::make());
          tree_->graft(".net." + netName + ".tx_bytes", Tree::make());
          tree_->graft(".net." + netName + ".tx_packets", Tree::make());
          tree_->graft(".net." + netName + ".tx_errors", Tree::make());
          tree_->graft(".net." + netName + ".tx_drop", Tree::make());
          _net[netName] = Net();
        } else {
          long rx_delta = rBytes - _net[netName].rx_bytes;
          long tx_delta = tBytes - _net[netName].tx_bytes;
          _net[netName].rx_rate = (long) ((double) rx_delta / period_);
          _net[netName].tx_rate = (long) ((double) tx_delta / period_);
          tree_->set_data(".net." + netName + ".rx_rate",
                          _net[netName].rx_rate);
          tree_->set_data(".net." + netName + ".tx_rate",
                          _net[netName].tx_rate);
        }
        _net[netName].rx_bytes = rBytes;
        _net[netName].rx_packets = rPackets;
        _net[netName].rx_errors = rErrs;
        _net[netName].rx_drop = rDrop;
        _net[netName].tx_bytes = tBytes;
        _net[netName].tx_packets = tPackets;
        _net[netName].tx_errors = tErrs;
        _net[netName].tx_drop = tDrop;
        tree_->set_data(".net." + netName + ".rx_bytes",
                        _net[netName].rx_bytes);
        tree_->set_data(".net." + netName + ".rx_packets",
                        _net[netName].rx_packets);
        tree_->set_data(".net." + netName + ".rx_errors",
                        _net[netName].rx_errors);
        tree_->set_data(".net." + netName + ".rx_drop",
                        _net[netName].rx_drop);
        tree_->set_data(".net." + netName + ".tx_bytes",
                        _net[netName].tx_bytes);
        tree_->set_data(".net." + netName + ".tx_packets",
                        _net[netName].tx_packets);
        tree_->set_data(".net." + netName + ".tx_errors",
                        _net[netName].tx_errors);
        tree_->set_data(".net." + netName + ".tx_drop",
                        _net[netName].tx_drop);
      }
    }
    file.close();
    // Graft the data to the tree
    graft_tree(".top.", tree_);
}

void SystemUsage::setRefreshPeriod(double period, void *user_data) {
  SystemUsage *ctx = static_cast<SystemUsage *>(user_data);
  ctx->period_ = period;
}

double SystemUsage::getRefreshPeriod(void *user_data) {
  SystemUsage *ctx = static_cast<SystemUsage *>(user_data);
  return ctx->period_;
}

} // namespace switcher
