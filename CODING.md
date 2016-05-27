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

When merging your branch into develop:
* please specify who reviewed your code in your message (use none if nobody reviewed your code). 
* if your branch contains more than one commit, do not use fast-forward when merging and write a higher level message including the reviewer, for instance (from the develop branch):
~~~~~~~~~~~~~~~~~~~~
git merge --no-ff feature/serialization -m'adding data structure serialization, reviewer: jsoria'
~~~~~~~~~~~~~~~~~~~~
* if your branch has only one commit, merge it with fast-forward and rename your commit with the reviewer name, for instance (from the develop branch):
~~~~~~~~~~~~~~~~~~~~
# merge with fast forward
git merge feature/serialization
# add reviewer into your commit
git commit --amend
~~~~~~~~~~~~~~~~~~~~
