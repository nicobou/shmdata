CODING
======

Coding style
------------

We use Google C++ Style Guide, as decribed [here](https://google.github.io/styleguide/cppguide.html), with two excpetions:
* a function call’s arguments will either be all on the same line or will have one line each. 
* a function declaration’s or function definition’s parameters will either all be on the same line or will have one line each.

In switcher, you may find not compliant code, but newly introduced code must follow this guide. Note that you may find configuration file for your editor [here](https://github.com/google/styleguide).

It is possible to let git ensure that you are conforming to the standards by using pre-commit hooks and clang-format:
~~~~~~~~~~~~~~~~~~~~
sudo apt-get install clang-format

#Then in switcher's .git folder:
rm -rf hooks && ln -s ../.hooks hooks
~~~~~~~~~~~~~~~~~~~~


Contributing
------------

Please send your pull request at the [sat-metalab' github account](https://github.com/sat-metalab/switcher). If you do not know how to make a pull request, github provides some [help about collaborating on projects using issues and pull requests](https://help.github.com/categories/collaborating-on-projects-using-issues-and-pull-requests/).

Branching strategy with git
---------------------------

The [master](https://github.com/sat-metalab/switcher/tree/master) branch contains switcher releases. Validated new developments are into the [develop](https://github.com/sat-metalab/switcher/tree/master) branch.
