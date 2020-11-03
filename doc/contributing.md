CONTRIBUTING
======

Coding style
------------

We use Google C++ Style Guide, as described [here](https://google.github.io/styleguide/cppguide.html), with two exceptions:
* A function call’s arguments will either be all on the same line or will have one line each. 
* A function declaration’s or function definition’s parameters will either all be on the same line or will have one line each.

For python code, we use PEP8 style guide with maximum line length set to 120.

In switcher, you may find not compliant code, but newly introduced code must follow this guide. Note that you may find configuration file for your editor [here](https://github.com/google/styleguide).

It is possible to let git ensure that you are conforming to the standards by using pre-commit hooks and clang-format and autopep8:
```
sudo apt-get install clang-format python3-autopep8

#Then in switcher's .git folder:
rm -rf hooks && ln -s ../.hooks hooks
```


Contributing
------------

Please send your merge request at the [sat-metalab' gitlab account](https://gitlab.com/sat-metalab/switcher). If you do not know how to make a merge request, gitlab provides some [help about creating a merge request](https://docs.gitlab.com/ee/gitlab-basics/add-merge-request.html).

Branching Strategy With Git
---------------------------

The [master](https://gitlab.com/sat-metalab/switcher/tree/master) branch contains switcher releases. Validated new developments are into the [develop](https://gitlab.com/sat-metalab/switcher/tree/develop) branch.

Modifications are made in a dedicated branch that needs to be merged into the develop branch through a gitlab merge request. When your modification is ready, you need to prepare your merge request as follows:
* Update your develop branch. 
```
git fetch
git checkout develop
git pull origin develop
```
* Go back to your branch and rebase onto develop. Note that it would be appreciated that you only merge request from a single commit (i.e interactive rebase with squash and/or fixup before pushing).
```
git rebase -i develop
```

Updating python and apt dependencies
------------------------------------

Lists of names for apt packages and python3 modules are maintained into separate files into the `deps` directory. When updating switcher dependencies, you must update these files along with the merge request that requires this update. Following this process ensures the update is propagated to:

* Install instructions in doc/INSTALL.md
* Continuous integration stages in gitlab

For apt there are several files, build and runtime dependencies, one for each supported distribution. Build dependencies are required for building switcher while runtime dependencies are required when running switcher. For instance, `libportmidi-dev` goes into the build file while `libportmidi0` goes into the runtime file. This separation is required for switcher packaging, like docker image building for instance.