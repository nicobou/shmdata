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

#undef NDEBUG  // get assert in release mode

#include <algorithm>
#include <cassert>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "switcher/infotree/information-tree-basic-serializer.hpp"
#include "switcher/infotree/information-tree-json.hpp"
#include "switcher/infotree/information-tree.hpp"

//----------------- a custom struct without operator <<
struct Widget : public DefaultSerializable<Widget> {};

//----------------- a custom struct with operator <<
struct SerializableWidget {
  friend std::ostream& operator<<(std::ostream& os, const SerializableWidget&);
};
std::ostream& operator<<(std::ostream& os, const SerializableWidget&) {
  os << "hello";
  return os;
}

//---------------- test
int main() {
  using namespace switcher;

  auto string_compare = [](const std::string& first, const std::string& second) {
    return (0 == first.compare(second));
  };

  {  // node data as std::string
    InfoTree::ptr tree = InfoTree::make(std::string("truc"));
    assert(tree->is_leaf());
    std::string data = tree->get_value();
    assert(0 == data.compare("truc"));
  }
  {  // node data as const char * (converted to std::string)
    InfoTree::ptr tree = InfoTree::make("test");
    assert(tree->is_leaf());
  }
  {  // node data as float
    InfoTree::ptr tree = InfoTree::make(1.2f);
    float val = tree->get_value();
    assert(1.2f == val);
  }
  {  // graft a multiple childs and get them
    InfoTree::ptr tree = InfoTree::make();
    tree->graft("...child1....child2..", InfoTree::make(1.2f));
    assert(!tree->is_leaf());
    InfoTree::ptr child2 = tree->get_tree("..child1.child2");
    assert(child2);
    assert(child2->is_leaf());
    float data = child2->get_value();
    assert(1.2f == data);
    InfoTree::ptr child1 = tree->get_tree(".child1..");
    assert(!child1->is_leaf());
    assert(tree->get_tree("child1.foo")->empty());  // this is not a child
  }
  {  // graft a childs and prune it
    InfoTree::ptr tree = InfoTree::make();
    tree->graft("child1.child2", InfoTree::make());
    assert(!tree->is_leaf());
    InfoTree::ptr child1 = tree->prune("child1");
    assert(child1);
    assert(tree->is_leaf());
    assert(!child1->is_leaf());
    // child2 from the pruned child1
    InfoTree::ptr child2 = child1->prune("child2");
    assert(child2);
    assert(child1->is_leaf());
    assert(child2->is_leaf());
  }
  {  // absolute prune
    InfoTree::ptr tree = InfoTree::make();
    tree->graft("child1.child2", InfoTree::make());
    InfoTree::ptr child2 = tree->prune("child1.child2");
    assert(!tree->is_leaf());
    assert(tree->branch_is_leaf("child1"));
  }
  {  // is_leaf with path
    InfoTree::ptr tree = InfoTree::make();
    tree->graft("child1", InfoTree::make());
    tree->graft("child1.child2", InfoTree::make());
    tree->graft("child1.child2.child3", InfoTree::make());
    assert(tree->branch_is_leaf("child1.child2.child3"));
    assert(!tree->branch_is_leaf("child1.child2"));
    assert(!tree->branch_is_leaf("child1"));
    assert(!tree->branch_is_leaf("foofoo"));
  }
  {  // set/get data with path
    InfoTree::ptr tree = InfoTree::make();
    // tree->set_value ("", 1.2f);  // this is not possible
    // float tree_data = tree->get_value (".");  // this is not possible
    // assert (1.2f == tree_data);
    tree->graft("child1.child2", InfoTree::make());
    assert(tree->branch_set_value("child1.child2", nullptr));
    assert(tree->branch_set_value("child1", 1.2f));
    std::string child2_data = tree->branch_get_value("child1.child2");
    assert("" == child2_data);
    float child1_data = tree->branch_get_value("child1");
    assert(1.2f == child1_data);
  }
  {  // removing using empty data
    InfoTree::ptr tree = InfoTree::make();
    tree->set_value("test");
    assert(tree->has_data());
    tree->set_value(Any());
    assert(!tree->has_data());
    tree->graft("child1.child2", InfoTree::make());
    assert(tree->branch_set_value("child1.child2", "test"));
    assert(tree->branch_set_value("child1", "test"));
    assert(tree->branch_has_data("child1.child2"));
    assert(tree->branch_has_data("child1"));
    assert(tree->branch_set_value("child1.child2", Any()));
    assert(tree->branch_set_value("child1", Any()));
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
    // std::cout << ss.str () << '\n';
    assert(0 == ss.str().compare("null-test-1.200000-not serializable-hello"));
  }
  {  // basic serialization
    InfoTree::ptr tree = InfoTree::make();
    tree->graft(".child1.child2", InfoTree::make("switch"));
    tree->graft(".child1.child3", InfoTree::make(1.2f));
    tree->graft(".child1.child2.bla1", InfoTree::make("wire"));
    tree->graft(".child1.child2.bla2", InfoTree::make("hub"));

    std::string serialized = infotree::keyval::serialize(tree.get());
    // std::cout << serialized << '\n';

    InfoTree::ptr tree2 = infotree::keyval::deserialize(serialized);
    assert(tree2);
    std::string serialized2 = infotree::keyval::serialize(tree2.get());
    // std::cout << serialized2 << '\n';

    assert(serialized == serialized2);
  }
  {
    // JSON serialization with child1.child2 as an array
    InfoTree::ptr tree = InfoTree::make();

    // if children are grafted to child3, its value
    // will be serialized with "key_value, as follow:
    // "child3" : {
    //  "key_value" : 1.2,
    //  "child4" : "float"
    //  }
    tree->graft(".child1.child3", InfoTree::make(1.2f));
    tree->graft(".child1.child3.child4", InfoTree::make("a string value"));
    tree->graft(".int_type", InfoTree::make(9));
    tree->graft(".bool_type", InfoTree::make(true));
    tree->graft(".float_type", InfoTree::make(3.14));
    tree->graft(".null_type", InfoTree::make());

    // if the array is made with a value, it will not be serialized
    // a node trageted as an array should be created like this :
    // tree->graft (".child1.child2", InfoTree::make ());
    // however, this is accepted:
    tree->graft(".child1.child2", InfoTree::make("switch"));
    //
    InfoTree::ptr elem1 = InfoTree::make();
    elem1->graft("equipment", InfoTree::make("door"));
    elem1->graft("color", InfoTree::make("red"));
    InfoTree::ptr elem2 = InfoTree::make();
    elem2->graft("equipment", InfoTree::make("door"));
    elem2->graft("color", InfoTree::make("black"));
    tree->graft(".child1.child2.red_door", elem1);
    tree->graft(".child1.child2.black_door", elem2);
    tree->tag_as_array(".child1.child2", true);

    std::string serialized = infotree::json::serialize(tree.get());
    // std::cout << serialized << '\n';
    auto deserialized_tree = infotree::json::deserialize(serialized);
    auto deserialized_string = infotree::json::serialize(deserialized_tree.get());
    // std::cout << deserialized_string << '\n';
    assert(serialized == deserialized_string);

    // test copy
    auto tree_cpy = InfoTree::copy(tree.get());
    assert(serialized == infotree::json::serialize(tree_cpy.get()));
    auto tree_cpy2 = tree->branch_get_copy(".");
    assert(serialized == infotree::json::serialize(tree_cpy2.get()));
  }

  {  // get childs keys inserting in an existing container
    InfoTree::ptr tree = InfoTree::make();
    std::list<std::string> childs{
        "child1", "child2", "child3", "child4", "child5", "child6", "child7", "child8", "child9"};
    std::for_each(childs.begin(), childs.end(), [tree](const std::string& val) {
      tree->graft(".root." + val, InfoTree::make("val"));
    });
    std::vector<std::string> child_keys;
    auto insert_it = std::insert_iterator<decltype(child_keys)>(child_keys, child_keys.begin());
    tree->copy_and_insert_child_keys(".root", insert_it);
    assert(std::equal(childs.begin(),
                      childs.end(),
                      child_keys.begin(),
                      [](const std::string& first, const std::string& second) {
                        return (0 == first.compare(second));
                      }));
  }

  {  // get childs keys in a newly allocated container
    InfoTree::ptr tree = InfoTree::make();
    std::list<std::string> childs{
        "child1", "child2", "child3", "child4", "child5", "child6", "child7", "child8", "child9"};
    std::for_each(childs.begin(), childs.end(), [tree](const std::string& val) {
      tree->graft(".root." + val, InfoTree::make("val"));
    });

    // using a default container
    auto child_keys_list = tree->get_child_keys(".root");
    assert(std::equal(childs.begin(), childs.end(), child_keys_list.begin(), string_compare));
  }

  {  // copy_leaf_values
    InfoTree::ptr tree = InfoTree::make();
    std::list<std::string> original_values{"0", "1", "2"};
    for (auto& it : original_values)
      tree->graft(std::string(".branch.item" + it), InfoTree::make(it));
    tree->graft(".other.branch", InfoTree::make());  // not into copy_leaf_values
    tree->tag_as_array("branch.", true);
    // std::string serialized = infotree::json::serialize(tree);
    // std::cout << serialized << '\n';
    std::list<std::string> values = tree->copy_leaf_values(".branch");
    assert(
        std::equal(original_values.begin(), original_values.end(), values.begin(), string_compare));
    // for (auto &it : values)
    //   std::cout << it << '\n';
  }

  {  // collecting data
    InfoTree::ptr tree = InfoTree::make();
    tree->graft("root.1.id", InfoTree::make(0));
    tree->graft("root.2.1.id", InfoTree::make(2));
    tree->graft("root.3.21.id", InfoTree::make(4));
    tree->graft("root.4.id", InfoTree::make(8));
    tree->graft("root.5.id", InfoTree::make(16));
    tree->graft("root.5.id.id", InfoTree::make(1));  // subtree of root.5.id
    tree->graft("root.1.2.id", InfoTree::make(3));   // sibling of root.1.id
    auto collected =
        InfoTree::collect_values(tree.get(),
                                 [](const std::string& key, InfoTree::ptrc) { return key == "id"; },
                                 false);  // do not continue search on siblings and subtree
    assert(5 == collected.size());
    std::vector<int> values{0, 2, 4, 8, 16};
    for (auto& it : collected) {
      assert(values.end() != std::find(values.cbegin(), values.cend(), it.copy_as<int>()));
    }
  }

  {  // graft by value
    InfoTree::ptr tree = InfoTree::make();
    tree->vgraft(".string", "a string value");
    tree->vgraft(".int", 9);
    tree->vgraft(".bool", true);

    auto tree2 = infotree::json::deserialize(
        R"({"string" : "a string value",
            "int" : 9,
            "bool" : true
           })");
    assert(infotree::json::serialize(tree.get()) == infotree::json::serialize(tree2.get()));
  }

  return 0;
}
