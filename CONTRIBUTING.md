# Contributing

Welcome! We'd love for you to contribute to Shmdata's code.

Shmdata is currently being maintained by a small team of developers working for the [Société des arts technologiques [SAT]](https://sat.qc.ca/), a non-profit art centre based in Montréal, Canada for which this software was originally developed. This core team adds features and bug fixes according to the SAT's interests and needs; however, community contributions on this project are more than welcome! There are many ways to contribute, including submitting bug reports, improving documentation, adding unit tests, adding support for a new language, submitting feature requests, reviewing new submissions, or contributing code that can be incorporated into the project.

This document describes this project's development process. Please do your best to follow these guidelines, as doing so will ensure a better contributing experience for you, and for other contributors and maintainers of this project.

## Code of Conduct

By participating in this project, you agree to abide by the Shmdata [Code of Conduct](CODE_OF_CONDUCT.md). We expect all contributors to follow the [Code of Conduct](CODE_OF_CONDUCT.md) and to treat fellow humans with respect. It must be followed in all your interactions with the project.

## Important Resources

* [Shmdata README](README.md)
* [Issue tracker](https://gitlab.com/sat-mtl/tools/shmdata/-/issues)

## Questions

The preferred way of asking question to us is through sending an email at metalab-dev@sat.qc.ca. We'll get back to you as soon as possible!

## Feature Requests

Please create a new Gitlab issue, using the "feature_request" description template, for a new feature that you wish to contribute or see added to the project. Please provide the feature you would like to see, why it is important to you, and how it will work. Discuss your ideas transparently and get developer feedback before proceeding. 

Small changes can directly be crafted and submitted to the repository as a Merge Request. See the section about the [Merge Request Process](#merge-request-process).

## Major changes and enhancements

Please create a new Gitlab issue, using the "rfc" description template, for a major changes or enhancement to the project. Such major changes should be discussed first in a Gitlab issue.


## Reporting Bugs

**If you find a security vulnerability, do NOT open an issue. Email metalab-dev@sat.qc.ca instead.**

Before you submit your issue, please [search the issue tracker](https://gitlab.com/sat-mtl/tools/shmdata/-/issues) - maybe your question or issue has already been identified or addressed.

If you find a bug in the source code, you can help us by [submitting an issue to the GitLab issue tracker](https://gitlab.com/sat-mtl/tools/shmdata/-/issues). In the issue, the bug must be described using the "bug_report" description template. Then, you can submit a Merge Request with a fix adding "close #10" in the merge request description (10 being here the example bug number, it must be replaced with the actual bug_report number). 

Please include as much information as possible in your issue: branch name and version, OS version, expected and observed behaviour, system information, log files, etc. Most importantly, provide a step-by-step procedure on how to reproduce your bug.

## Adding tests

You are welcome to add unit tests if you spot some code that isn't covered by the existing tests.

We do not expect you to add unit tests when doing only minor changes to the codebase. We expect, however, the tests to pass when you're finished with your work.

When adding a new feature or doing major changes to the code, we expect you to add the relevant unit tests to the code.

In any case, please run the tests on your computer first and ensure that they pass before pushing them to the repository. The CI pipeline will spot any broken tests, but we prefer that you make the necessary verifications beforehand rather than making the CI fail needlessly.

### Running Tests

From the shmdata build directory, you can run tests with the following command:

```bash
  make test
```

## Improving Documentation

Should you have a suggestion for the documentation, you can open an issue and outline the problem or improvement you have - however, creating the fix yourself is much better!

If you want to help improve the docs and it's a substantial change, create a new issue (or comment on a related existing one) to let others know what you're working on. Small changes (typos, improvements to phrasing) do not need an issue.


For large fixes, please build and test the documentation before submitting the MR to be sure you haven't accidentally introduced any layout or formatting issues.

## Contributing Code

### Finding an Issue

The list of outstanding feature requests and bugs can be found on the [GitLab issue tracker](https://gitlab.com/sat-mtl/tools/shmdata/-/issues). Pick an unassigned issue that you think you can accomplish and add a comment that you are attempting to do it.

### Development Process

This project follows the [git flow](http://nvie.com/posts/a-successful-git-branching-model/) branching model of product development.

The **master** branch contains the latest stable production release, and is updated by the core developer team. Each commit in the **master** branch is in fact a production release and is tagged as such. The **develop** branch contains the latest development version of the project.

When contributing to the project, you should create a new *feature branch* based off the **develop** branch.

Always give a short but meaningful name to this new branch, and follow these conventions:

* A branch that aims to fix a bug should be named *fix/\<branchname\>*
* A branch that aims to add a new feature should be named *feat/\<branchname\>*
* A branch that aims to change documentation should be named *doc/\<branchname\>*
* A branch that aims to add tests should be named *test/\<branchname\>*
* The branch cannot be named *master*, *develop*, *release/\** or *hotfix/\**

When your feature branch is ready, submit a Merge Request to **develop**. When the MR is approved and your changes merged, you can safely delete your feature branch.

Core developers will prepare new releases regularly by creating a *release branch* from **develop** (named *release/<new_version>*). This new branch allows developers to test the release's stability and prepare the relevant metadata. Eventually, when this new release has been tested and deemed stable enough, it will be merged in the **master** branch by the core team. This new commit on **master** will be tagged for future reference and the original release branch will then be deleted.

In the event that a severe bug is detected on **master**, a special *hotfix branch* will be created (named *hotfix/<new_version>*). This hotfix branch can only be created and merged by core developers from the **master** branch.


### CI pipeline

This repository includes a CI pipeline, used to validate that incoming commits do not break the build, and that all unit tests still pass. It will be run automatically when a new commit is pushed to the repository. If the pipeline fails, it is your responsibility to checkout the [CI Jobs page](https://gitlab.com/sat-mtl/tools/shmdata/-/jobs) and figure out how to fix it. A Merge Requests that breaks the pipeline will not be merged by the core developers.

## Merge Request Process

In order to propose a merge request, please follow the gitlab recommended process: fork, mirror & merge request. This process is documented [here](https://docs.gitlab.com/ee/user/project/repository/forking_workflow.html).

When you are ready to submit you changes for review, either for preliminary review or for consideration of merging into the project, you can create a Merge Request to add your changes into **develop**. Only core developers are allowed to merge your branch into **develop**.

Do not forget to:

1. Update the relevant [README.md](README.md) sections, if necessary.
2. Add the relevant unit tests, if needed.
3. Update documentation, if needed.
4. Rebase your changes in nice, clear commits if your branch's commit history is messy.

A core developer will merge your changes into **develop** when all feedback have been addressed.

### Review Process

The core developer team regularly checks the repository for new merge requests. We expect the CI pipeline to succeed before approval of the MR. If it doesn't, your MR will not be merged. 

If the MR concerns a minor issue, it will be merged immediately after one of the core developers approves it, if the CI succeeds. For other issues, we will wait a day or two after all feedback has been addressed before merging the request.

### Addressing Feedback

Once a MR has been submitted, your changes will be reviewed and constructive feedback may be provided. Feedback isn't meant as an attack, but to help make sure the highest-quality code makes it into the project. Changes will be approved once required feedback has been addressed.

## License

By contributing to this repository, you agree that your contributions will be licensed in accordance to the [LICENSE](LICENSE.md) document at the root of this repository.



## Coding style

We use Google C++ Style Guide, as described [here](https://google.github.io/styleguide/cppguide.html), with two exceptions:
* A function call’s arguments will either be all on the same line or will have one line each. 
* A function declaration’s or function definition’s parameters will either all be on the same line or will have one line each.

For python code, we use PEP8 style guide with maximum line length set to 120.
