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
 */

#include "./net-utils.hpp"
#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <resolv.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unistd.h>

namespace switcher {

bool NetUtils::is_used(std::uint16_t port) {
  bool res = true;
  struct addrinfo hints;
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
  hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
  hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
  hints.ai_protocol = 0;          /* Any protocol */
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;
  struct addrinfo* result;
  int s = getaddrinfo(NULL, std::to_string(port).c_str(), &hints, &result);
  if (s != 0) {
    return false;
  }
  /* getaddrinfo() returns a list of address structures.
     Try each address until we successfully bind(2).
     If socket(2) (or bind(2)) fails, we (close the socket
     and) try the next address. */
  struct addrinfo* rp;
  for (rp = result; rp != NULL; rp = rp->ai_next) {
    int sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (sfd == -1) continue;
    if (bind(sfd, rp->ai_addr, rp->ai_addrlen) == 0) {
      res = false;
    }
    close(sfd);
  }
  freeaddrinfo(result);
  return res;
}

std::map</* interface name */ std::string, /* ip */ std::string> NetUtils::get_ips() {
  std::map<std::string, std::string> res;
  struct ifaddrs *ifaddr, *ifa;
  int family, s;
  char host[NI_MAXHOST];
  if (getifaddrs(&ifaddr) == -1) {
#ifdef DEBUG
    int err = errno;
    std::cerr << "getifaddrs errors: " << strerror(err)) << '\n';
#endif
    return res;
  }
  /* Walk through linked list, maintaining head pointer so we
     can free list later */
  for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == NULL) continue;
    family = ifa->ifa_addr->sa_family;
    // /* Display interface name and family (including symbolic
    //    form of the latter for the common families) */
    // printf("%s  address family: %d%s\n",
    //        ifa->ifa_name, family,
    //        (family == AF_PACKET) ? " (AF_PACKET)" :
    //        (family == AF_INET) ?   " (AF_INET)" :
    //        (family == AF_INET6) ?  " (AF_INET6)" : "");
    /* For an AF_INET* interface address, display the address */
    if (family == AF_INET /*|| family == AF_INET6*/) {
      s = getnameinfo(
          ifa->ifa_addr,
          (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6),
          host,
          NI_MAXHOST,
          NULL,
          0,
          NI_NUMERICHOST);
      if (s != 0) {
#ifdef DEBUG
        std::cerr << "getnameinfo() failed: " << gai_strerror(s) << '\n';
#endif
        return res;
      }
      res[ifa->ifa_name] = std::string(host);
    }
  }
  freeifaddrs(ifaddr);
  return res;
}

std::string NetUtils::get_system_dns() {
  if (res_init()) {
    return kDefaultDNS;
  }

  char dns_addr[64];
  inet_ntop(AF_INET, &_res.nsaddr.sin_addr.s_addr, dns_addr, 64);
  return dns_addr;
}

bool NetUtils::is_valid_IP(const std::string& ip) {
  struct sockaddr_in sa;
  return inet_pton(AF_INET, ip.c_str(), &sa.sin_addr) == 1;
}

}  // namespace switcher
