CONTRIBUTING
======

Welcome! We'd love for you to contribute to Switcher.

Switcher is currently being maintained by a small team of developers working for the [Société des arts technologiques [SAT]](https://sat.qc.ca/), a non-profit art centre based in Montreal, Canada for which this software was originally developed. This core team adds features and bug fixes according to the SAT's interests and needs; however, community contributions on this project are more than welcome! There are many ways to contribute, including submitting bug reports, improving documentation, adding unit tests, adding support for a new language, submitting feature requests, reviewing new submissions, or contributing code that can be incorporated into the project.

This document describes this project's development process. Please do your best to follow these guidelines, as doing so will ensure a better contributing experience for you, and for other contributors and maintainers of this project.

Code of Conduct
---------------

By participating in this project, you agree to abide by the Switcher [Code of Conduct](./code-of-conduct.md). We expect all contributors to follow the [Code of Conduct](./code-of-conduct.md) and to treat fellow humans with respect. It must be followed in all your interactions with the project.


Contributing
------------

Contributing to switcher is achieved through Gitlab's Merge Request (MR) system. This include contribution by the core team. We also welcome external contributions through the contribution process described here.

Please send your merge request to [switcher repository](https://gitlab.com/nicobou/switcher). If you do not know how to make a merge request, gitlab provides some [help about creating a merge request](https://docs.gitlab.com/ee/gitlab-basics/add-merge-request.html).

Merge request must refer to a gitlab issue in the [switcher issues list](https://gitlab.com/nicobou/switcher/-/issues). If your merge request does not refer to a gitlab issue, you may be asked to fill an issue before a decision is made. 

Switcher has several issue types:

- Default issue: for all issues that do not match with following issue types.
- Bug report: inform of a newly discovered bug.
- Feature request: ask for a new feature in switcher.
- Request For Comment: propose a significant change in switcher, such as code refactoring, CI deployment, or any change in the repository that impacts the switcher community. See a more detailled description of the [RFC process](./rfc.md).

Migration
---------

Breaking changes applied to the Switcher API are documented in our [migration file](MIGRATIONS.md). 

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

Branching Strategy With Git
---------------------------

The [master](https://gitlab.com/nicobou/switcher/tree/master) branch contains switcher releases. Validated new developments are into the [develop](https://gitlab.com/nicobou/switcher/tree/develop) branch.

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


