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

#include "switcher/make-consultable.hpp"
#include <string>
#include <iostream>

class Door {
 public:
  Door(std::string str):
      name_(std::move(str)){
  }
  // -------- read only methods (const)
  std::string get_name() const {
    return name_;
  }
  std::string get_name(int num) const {
    return name_ + " " + std::to_string(num);
  }
  std::string copy_and_append(int num, std::string str) const {
    return name_ + " " + std::to_string(num) + " " + str;
  }
  void copy_append_and_print(int num, std::string str) const {
    std::printf("%s\n", std::string(name_
                                    + " "
                                    + std::to_string(num)
                                    + " " + str).c_str());
  }
  template<typename T> std::string append_templ(T val) const {
    return name_ + " " + std::to_string(val);
  }

  template<typename T> void print_templ(T val) const {
    std::printf("%s\n", std::string(name_ + " " + std::to_string(val)).c_str());
  }

  // ---------- read/write methods (non-const)
  bool set_name(std::string str) {
    name_ = std::move(str);
    return true;
  }
  void set_name_no_return(std::string str) {
    name_ = std::move(str);
  }

 private:
  std::string name_{"Nico"};
};

class Room {
 public:
  Make_consultable(Door, &left_door_, ldoor);
  Make_consultable(Door, &right_door_, rdoor);

 private:
  Door right_door_{"right"};
  Door left_door_{"left"};
};

int main() {
  Room r{};

  { // no need to give args if no overload is creating ambiguity
    auto res =
        r.rdoor<std::string>(&Door::copy_and_append, 3, std::string("coucou"));
    std::printf("%s\n", res.c_str());
  }

  { // using method returning void
    r.rdoor<void>(&Door::copy_append_and_print, 4, std::string("hello"));
  }
  
  {  // using rdoor_.get_name() 
    auto res =
        r.rdoor<std::string>(&Door::get_name);
    std::printf("%s\n", res.c_str());
  }

  { // using rdoor_.get_name(int)
    // arg type (int) is given to the template in order to
    // select the right overload
    auto res =
        r.rdoor<std::string, int>(&Door::get_name, 3);
    std::printf("%s\n", res.c_str());
  }

  { // -do not use this- (verbosely) using rdoor_.get_name()
    // but the template arg is the return type only
    auto res =
        r.rdoor<std::string>(
            static_cast<std::string(Door::*)(int) const>(&Door::get_name),
            3);
    std::printf("%s\n", res.c_str());
  }

  { // template member
    // pass arg type in order to get template instanciation
    auto res =
        r.rdoor<std::string, double>(&Door::append_templ<double>, 3.14);
    std::printf("%s\n", res.c_str());
  }

  { // template returning void
    // pass arg type in order to get template instanciation
    r.rdoor<void, double>(&Door::print_templ<double>, 2.71);
  }

  // { // fail to compile since set_name is not const
  //   bool res =
  //       r.rdoor<bool>(&Door::set_name, std::string("coucou"));
  //   std::printf("%s\n", std::to_string(res).c_str());
  // }

  // { // fail to compile since set_name is not const
  //   r.rdoor<>(&Door::set_name, std::string("coucou"));
  //   std::printf("%s\n", std::to_string(res).c_str());
  // }

  // ------------------------------------------
  // ------------------ the same with ldoor
  // ------------------------------------------
  { // no need to give args if no overload is creating ambiguity
    auto res =
        r.ldoor<std::string>(&Door::copy_and_append, 3, std::string("coucou"));
    std::printf("%s\n", res.c_str());
  }

  { // using method returning void
    r.ldoor<void>(&Door::copy_append_and_print, 4, std::string("hello"));
  }
  
  {  // using ldoor_.get_name() 
    auto res =
        r.ldoor<std::string>(&Door::get_name);
    std::printf("%s\n", res.c_str());
  }

  { // using ldoor_.get_name(int)
    // arg type (int) is given to the template in order to
    // select the right overload
    auto res =
        r.ldoor<std::string, int>(&Door::get_name, 3);
    std::printf("%s\n", res.c_str());
  }

  { // -do not use this- (verbosely) using ldoor_.get_name()
    // but the template arg is the return type only
    auto res =
        r.ldoor<std::string>(
            static_cast<std::string(Door::*)(int) const>(&Door::get_name),
            3);
    std::printf("%s\n", res.c_str());
  }

  { // template member
    // pass arg type in order to get template instanciation
    auto res =
        r.ldoor<std::string, double>(&Door::append_templ<double>, 3.14);
    std::printf("%s\n", res.c_str());
  }
    
  return 0;
}
