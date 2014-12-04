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
#include <string>
#include <cassert>
#include <iostream>
#include <memory>
#include <algorithm>
#include <list>
#include <vector>

#include "switcher/information-tree.hpp"
#include "switcher/information-tree-basic-serializer.hpp"
#include "switcher/information-tree-json.hpp"

//----------------- a custom struct without operator <<
struct Widget: public DefaultSerializable<Widget> {
};

//----------------- a custom struct with operator <<
struct SerializableWidget {
  friend std::ostream &operator<<(std::ostream &os,
                                  const SerializableWidget &);
};
std::ostream &operator<<(std::ostream &os, const SerializableWidget &) {
  os << "hello";
  return os;
}

//---------------- test
int
main() {
  using namespace switcher::data;

  auto string_compare =
      [](const std::string &first, const std::string &second)
      { return (0 == first.compare(second)); };

  {  // node data as std::string
    Tree::ptr tree = Tree::make(std::string("truc"));
    assert(tree->is_leaf());
    std::string data = tree->get_data();
    assert(0 == data.compare("truc"));
  }
  {  // node data as const char * (converted to std::string)
    Tree::ptr tree = Tree::make("test");
    assert(tree->is_leaf());
  }
  {  // node data as float
    Tree::ptr tree = Tree::make(1.2f);
    float val = tree->get_data();
    assert(1.2f == val);
  }
  {  // graft a multiple childs and get them
    Tree::ptr tree = Tree::make();
    tree->graft("...child1....child2..", Tree::make(1.2f));
    assert(!tree->is_leaf());
    Tree::ptr child2 = tree->get("..child1.child2");
    assert(child2);
    assert(child2->is_leaf());
    float data = child2->get_data();
    assert(1.2f == data);
    Tree::ptr child1 = tree->get(".child1..");
    assert(!child1->is_leaf());
    assert(!tree->get("child1.foo"));   // this is not a child
  }
  {  // graft a childs and prune it
    Tree::ptr tree = Tree::make();
    tree->graft("child1.child2", Tree::make());
    assert(!tree->is_leaf());
    Tree::ptr child1 = tree->prune("child1");
    assert(child1);
    assert(tree->is_leaf());
    assert(!child1->is_leaf());
    // child2 from the pruned child1
    Tree::ptr child2 = child1->prune("child2");
    assert(child2);
    assert(child1->is_leaf());
    assert(child2->is_leaf());
  }
  {  // is_leaf with path
    Tree::ptr tree = Tree::make();
    tree->graft("child1", Tree::make());
    tree->graft("child1.child2", Tree::make());
    tree->graft("child1.child2.child3", Tree::make());
    assert(tree->branch_is_leaf("child1.child2.child3"));
    assert(!tree->branch_is_leaf("child1.child2"));
    assert(!tree->branch_is_leaf("child1"));
    assert(!tree->branch_is_leaf("foofoo"));
  }
  {  // set/get data with path
    Tree::ptr tree = Tree::make();
    // tree->set_data ("", 1.2f);  // this is not possible
    // float tree_data = tree->get_data (".");  // this is not possible
    // assert (1.2f == tree_data);
    tree->graft("child1.child2", Tree::make());
    assert(tree->set_data("child1.child2", "test"));
    assert(tree->set_data("child1", 1.2f));
    std::string child2_data = tree->get_data("child1.child2");
    assert(0 == child2_data.compare("test"));
    float child1_data = tree->get_data("child1");
    assert(1.2f == child1_data);
  }
  {  // removing using empty data
    Tree::ptr tree = Tree::make();
    tree->set_data("test");
    assert(tree->has_data());
    tree->set_data(Any());
    assert(!tree->has_data());
    tree->graft("child1.child2", Tree::make());
    assert(tree->set_data("child1.child2", "test"));
    assert(tree->set_data("child1", "test"));
    assert(tree->branch_has_data("child1.child2"));
    assert(tree->branch_has_data("child1"));
    assert(tree->set_data("child1.child2", Any()));
    assert(tree->set_data("child1", Any()));
    assert(!tree->branch_has_data("child1.child2"));
    assert(!tree->branch_has_data("child1"));
  }
  {  // Any to string
    Any n;
    Any a(std::string("test"));
    Any b(1.2f);
    Widget widget;
    Any w(std::move(widget));
    SerializableWidget serializable_widget;
    Any sw(serializable_widget);

    std::stringstream ss;
    ss << n << "-" << a << "-" << b << "-" << w << "-" << sw;
    // std::cout << ss.str () << std::endl;
    assert(0 == ss.str().compare("null-test-1.2-not serializable-hello"));
  }
  {  // basic serialization
    Tree::ptr tree = Tree::make();
    tree->graft(".child1.child2", Tree::make("switch"));
    tree->graft(".child1.child3", Tree::make(1.2f));
    tree->graft(".child1.child2.bla1", Tree::make("wire"));
    tree->graft(".child1.child2.bla2", Tree::make("hub"));

    std::string serialized = BasicSerializer::serialize(tree.get());
    std::cout << serialized << std::endl;

    Tree::ptr tree2 = BasicSerializer::deserialize(serialized);
    assert(tree2);
    std::string serialized2 = BasicSerializer::serialize(tree2.get());
    std::cout << serialized2 << std::endl;

    assert(serialized == serialized2);
  }
  {
    // JSON serialization with child1.child2 as an array
    Tree::ptr tree = Tree::make();

    // if children are grafted to child3, its value
    // will be serialized with "key_value, as follow:
    // "child3" : {
    //  "key_value" : "1.2",
    //  "child4" : "float"
    //  }
    tree->graft(".child1.child3", Tree::make(3.1f));
    tree->graft(".child1.child3.child4", Tree::make("child4_value"));

    // if the array is made with a value, it will not be serialized
    // a node trageted as an array should be created like this :
    // tree->graft (".child1.child2", Tree::make ());
    // however, this is accepted:
    tree->graft(".child1.child2", Tree::make("switch"));
    //
    Tree::ptr elem1 = Tree::make();
    elem1->graft("equipment", Tree::make("door"));
    elem1->graft("color", Tree::make("red"));
    Tree::ptr elem2 = Tree::make();
    elem2->graft("equipment", Tree::make("door"));
    elem2->graft("color", Tree::make("black"));
    tree->graft(".child1.child2.red_door", elem1);
    tree->graft(".child1.child2.black_door", elem2);
    tree->tag_as_array(".child1.child2", true);

    std::string serialized = JSONSerializer::serialize(tree.get());
    std::cout << serialized << std::endl;
  }

  {  // get childs keys inserting in an existing container
    Tree::ptr tree = Tree::make();
    std::list<std::string> childs {
      "child1", "child2", "child3",
          "child4", "child5", "child6", "child7", "child8", "child9"};
    std::for_each(childs.begin(),
                  childs.end(),
                  [tree] (const std::string &val) {
                    tree->graft(".root." + val, Tree::make("val"));
                  });
    std::vector<std::string> child_keys;
    auto insert_it =
        std::insert_iterator<decltype(child_keys)>(child_keys, child_keys.begin());
    tree->copy_and_insert_child_keys(".root",
                                     insert_it);
    assert(std::equal(childs.begin(),
                      childs.end(),
                      child_keys.begin(),
                      [](const std::string &first,
                         const std::string &second) {
                        return (0 == first.compare(second));
                      }));
  }

  {  // get childs keys in a newly allocated container
    Tree::ptr tree = Tree::make();
    std::list<std::string> childs { "child1", "child2", "child3", "child4",
          "child5", "child6", "child7", "child8", "child9"};
    std::for_each(childs.begin(),
                  childs.end(),
                  [tree] (const std::string &val) {
                    tree->graft(".root." + val, Tree::make("val"));
                  });

    // using a default container
    auto child_keys_list =
        tree->get_child_keys (".root");
    assert(std::equal
           (childs.begin(), childs.end(),
            child_keys_list.begin(),
            string_compare));

  }

  {  // copy_leaf_values
    Tree::ptr tree = Tree::make();
    std::list<std::string> original_values {"0", "1", "2"};
    for (auto &it : original_values)
      tree->graft(std::string(".branch.item"+it),
                              Tree::make(it));
    tree->graft(".other.branch", Tree::make());  // not into copy_leaf_values
    tree->tag_as_array("branch.", true);
    // std::string serialized = JSONSerializer::serialize(tree);
    // std::cout << serialized << std::endl;
    std::list<std::string> values = tree->copy_leaf_values(".branch");
    assert(std::equal(original_values.begin(), original_values.end(),
                      values.begin(),
                      string_compare));
    // for (auto &it : values)
    //   std::cout << it << std::endl;
  }

  return 0;
}
