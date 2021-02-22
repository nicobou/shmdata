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

Ubuntu packaging is derived from Debian (as Ubuntu is built on top of Debian). Configuration files related to this are located in a separate branch (namely `debian/master`) to prevent polluting the library sources. Usually Ubuntu packaging is done in a separate repository, but in our case since the [SAT]Metalab maintains the upstream (the sources) as well as the packaging, it is simpler to have everything in a single repository.

To summarize the packaging process:
* a tarball is created from the latest release branch, `master` in our case
* `master` is merged into `debian/master`
* Debian packaging tools are used to generate the binary package locally
* if it succeeds, the source package is generated
* the generated source package is uploaded to the PPA using the user's credentials on [Launchpad](https://launchpad.net)
* an email is sent to the user's address validating if the source package has been accepted or not
* if accepted, the build status can be checked from Launchpad. An email will be sent regarding the build status.

After the upload, the process can take some time depending on the availability of Launchpad. So please be patient as it can sometimes take a few hours...

### Updating the packages

To update the package, you need to install a few additionnal dependencies, as well as those needed for building the library:

```bash
sudo apt install debhelper dpkg-dev git-buildpackage
```

The process to update the package is as follows:

* Create or login to [Launchpad](https://login.launchpad.net). You need to be a member of the [SAT]Metalab team on Launchpad to be allowed to upload packages.
* Run the Ubuntu packaging script:

```bash
./scripts/package_ubuntu.py
```

  This script will clone shmdata in a temporary directory, merge `master` into `debian/master`, try buildding the packages locally and if it succeeds, build the source package and upload it to the PPA. Then it will commit the `debian/master` branch, generate a new tag `debian/${VERSION}` and push everything to the repository.

### Fixing issues

If the upload fails, the source package creation has to be done manually. This can happen when:
* the same version of the package has already been uploaded: usually this means that the package is already up-to-date on the PPA.
* a building dependency is missing in the `debian/control` file: in this case, just add the missing dependency in the `debian/master` branch, commit and push to the repository, and retry running the script.
* the building or testing fails during the binary package creation on Launchpad

This last case can (and probably will) be more tedious to fix. It means either fixing the problem in the upstream (`master` branch) if it can be reproduced locally, or creating a patch to allow for the build to succeed. The packaging is configured to use `quilt` to create patches, and `git-buildpackage` is used on top of it to maintain the patches through the successive merges of `master` into `debian/master`.

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
