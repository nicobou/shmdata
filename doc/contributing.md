CONTRIBUTING
============

Coding style
------------

We use Google C++ Style Guide, as described [here](https://google.github.io/styleguide/cppguide.html), with two exceptions:
* A function call’s arguments will either be all on the same line or will have one line each. 
* A function declaration’s or function definition’s parameters will either all be on the same line or will have one line each.

For python code, we use PEP8 style guide with maximum line length set to 120.

        
# Docker images

A docker image of `shmdata` is built into its [registry](https://gitlab.com/sat-metalab/shmdata/container_registry). You can start building your own library on top of this image by pulling it.

Three tags of this image are provided :

| tag                | purpose     | description                                                            |
|--------------------|-------------|------------------------------------------------------------------------|
| [master][master]   | production  | Clean image based on the `master` branch and build with `Release` flag |
| [develop][develop] | development | Clean image based on the `develop` branch and build with `Debug` flag  |
| [ci][ci]           | testing     | Image used for CI and used for unit tests                              |

[master]: registry.gitlab.com/sat-metalab/shmdata:master
[develop]: registry.gitlab.com/sat-metalab/shmdata:develop
[ci]: registry.gitlab.com/sat-metalab/shmdata:ci

## Install with Docker

1. Install Docker ([instructions for Ubuntu 18.04](https://docs.docker.com/install/linux/docker-ce/ubuntu/))

2. Pull the Docker image

```bash
docker pull registry.gitlab.com/sat-metalab/shmdata:develop # or use "master" tag
```

3. Configure Nvidia runtime (from [instructions](https://gitlab.com/sat-metalab/switcher/blob/develop/doc/run-switcher-in-docker.md#install-the-nvidia-docker-runtime-in-ubuntu-1804))

4. Develop your own tools from `shmdata`
    
    * You can take inspiration from [the Switcher and Scenic build instructions](https://gitlab.com/sat-metalab/switcher/blob/develop/doc/run-switcher-in-docker.md#run-scenic-from-remote-image-with-nvidia-support)
    
    * You can create your own Dockerfile

    ```bash
    FROM registry.gitlab.com/sat-metalab/shmdata
    # your Dockerfile
    ```

## Contribute with Docker

The `shmdata` image uses [mutli-stage builds][docker-multi-stage] with three stages : `dependencies`, `build` and `clean`. Theses stages use some build arguments :

| variables  | stages              | description                      | default        |
|------------|---------------------|----------------------------------|----------------|
| BUILD_DIR  | `build` and `clean` | Where `shmdata` source is copied | `/opt/shmdata` |
| BUILD_TYPE | `build`             | The build type of `shmdata`      | `Release`      |

All images can be built and tested from source :

```bash
# build shmdata with the "build" stage, all unused dependencies are not removed
docker build -t shmdata:test -f dockerfiles/Dockerfile --target build .

# execute bash into the BUILD_DIR folder
docker run -ti shmdata:test bash
```

[docker-multi-stage]: https://docs.docker.com/develop/develop-images/multistage-build/


# Packaging

shmdata is currently packaged for Debian and the latest LTS version of Ubuntu.

## Simple Debian package

Debian packages can be built directly from the sources:

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
make package
```

A package will be generated in `build/libshmdata_${VERSION}_${PLATFORM}.deb`. This package can be installed on other computers provided that the dependencies are met, and the plateform is the same.

## Ubuntu packaging

Packages for Ubuntu are built using their PPA (Private Package Archive) system. As of writing this documentation, shmdata is distributed through [SAT Metalab's PPA](https://launchpad.net/~sat-metalab/+archive/ubuntu/metalab)

### Overview

Ubuntu packaging is derived from Debian (as Ubuntu is buillt on top of Debian). Configuration files related to this are located in a separate branch (namely `debian/master`) to prevent polluting the library sources. Usually Ubuntu packaging is done in a separate repository, but in our case as the [SAT]Metalab is maintaining the upstream (the sources) as well as the packaging, it is simpler to have everything in a single repository.

To summarize the packaging process:
* a tarball is created from the latest release branch, `master` in our case
* `master` is merge into `debian/master`
* Debian packaging tools are used to generate the binary package locally
* if it succeeds, the source package is generated
* the generated source package is uploaded to the PPA using the user's credentials on [Launchpad](https://launchpad.net)
* an email is sent to the user's address validating that the source package has been accepted or not
* if accepted, the build status can be checked from Launchpad. An email will be sent regarding the build status.

After the upload, the process can take some time depending on the availability of Launchpad. So please be patient as it can sometimes take a few hours...

### Updating the packages

To update the package, you need to install a few dependencies, additionnally to dependencies needed for building the library:

```bash
sudo apt install debhelper dpkg-dev git-buildpackage
```

The process to update the package is as follows:

* Create or login to [Launchpad](https://login.launchpad.net). You need to be a member of the [SAT]Metalab team on Launchpad to be able to be allowed to upload packages.
* Run the Ubuntu packaging script:

```bash
./scripts/package_ubuntu.py
```

  This script will clone shmdata in a temporary directory, merge `master` into `debian/master`, try buildding the packages locally and if it succeeds, build the source package and upload it to the PPA. Then it will commit the `debian/master` branch, generate a new tag `debian/${VERSION}` and push all that to the repository.

### Fixing issues

If the upload fails, source package creation has to be done manually. This can happen when:
* the same version of the package has already been uploaded: usually this means that the package is already up-to-date on the PPA.
* a building dependency is missing in the `debian/control` file: in this case, just add the missing dependency in the `debian/master` branch, commit and push to the repository, and retry running the script.
* the building or testing fails during binary package creation on Launchpad

This last case can (and probably will) be more tedious to fix. It means either fixing the problem in the upstream (`master` branch) if it can be reproduced locally. Or it means creating a patch to allow for the build to succeed. The packaging is configured to used `quilt` to create patches, and `git-buildpackage` is used on top of it to maintain it through the successive merges of `master` into `debian/master`.

To create a patch:
```bash
git checkout debian/master
gbp pq import
# Fix the sources
# Commit the changes
gbp pq export
gbp pq drop
git add debian/patches
git commit
```

Then to allow for the patch to be applied at build-time:
```bash
quilt import debian/patches/xxxx-patch-name.patches
quilt push
quilt refresh
quilt header -e
git add debian/patches
git commit
```

Push the changes to the repository, and retry running the packaging script. Note that the packaging script maintains the patches through each merge with `master` by itself, if all goes well.
