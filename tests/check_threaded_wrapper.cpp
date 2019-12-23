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

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <iostream>
#include <string>
#include "switcher/utils/make-consultable.hpp"  // for MPtr
#include "switcher/utils/threaded-wrapper.hpp"

using namespace std;
using namespace switcher;

// struct that will be instanciaded and invoked in a specific thread
struct A {
  A() = default;
  A(const string& str) : str_(str) {}

  void do_nothing() {}

  string hello(int i, const string& msg) {
    if (0 == i) cout << str_ << to_string(i) << " " << msg << endl;
    return string("hello") + to_string(i);
  }

 private:
  string str_{};
};

// other threads using struct A
void use_A(ThreadedWrapper<A>& twa, const string& str) {
  int i = 100;
  while (--i >= 0) {
    auto res = twa.invoke<MPtr(&A::hello)>(i, str);
    assert(res == string("hello") + to_string(i));
    twa.invoke<MPtr(&A::do_nothing)>();
  }
}

void use_A_async(ThreadedWrapper<A>& twa, const string& str) {
  int i = 100;
  while (--i >= 0) {
    twa.invoke_async<MPtr(&A::hello)>(
        [=](const string& str) { assert(str == string("hello") + to_string(i)); }, i, str);
    twa.invoke_async<MPtr(&A::do_nothing)>([=]() {
      if (0 == i) cout << str << ": last do_nothing invocation done" << endl;
    });
  }
}

int main() {
  ThreadedWrapper<A> twa("coucou");
  thread th1(use_A, ref(twa), "th1 (sync)");
  thread th2(use_A, ref(twa), "th2 (sync)");
  thread th3(use_A_async, ref(twa), "th3 (async)");
  thread th4(use_A_async, ref(twa), "th4 (async)");
  th1.join();
  th2.join();
  th3.join();
  th4.join();
}
