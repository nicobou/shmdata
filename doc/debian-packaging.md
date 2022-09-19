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

Packages for Ubuntu are built using the Metalab [distribution repositories](https://gitlab.com/sat-mtl/distribution).

### Overview

Ubuntu packaging is derived from Debian (as Ubuntu is built on top of Debian). Configuration files related to this are located in a separate branch (namely `debian/master`) to prevent polluting the library sources. Usually Ubuntu packaging is done in a separate repository, but in our case since the [SAT] Metalab maintains the upstream (the sources) as well as the packaging, it is simpler to have everything in a single repository.

To summarize the packaging process (implemented in the `script/package_ubuntu.py`):
* Clone Shmdata in a new directory
* Merge master branch into debian/master
* Update debian/changelog with new package version
* Commit changes in the debian/master branch
* Ask the user if the updated debian/master branch needs to be pushed
* If pushed, the Shmdata continuous integration (CI) will build and publish the new package


After the upload, the process can take some time depending on the availability of the CI. So please be patient.

### Updating the packages

To update the package, run the Ubuntu packaging script:

```bash
./scripts/package_ubuntu.py
```

  This script will clone shmdata in a temporary directory, merge `master` into `debian/master`, try buildding the packages locally and if it succeeds, build the source package and upload it to the PPA. Then it will commit the `debian/master` branch, generate a new tag `debian/${VERSION}` and push everything to the repository.

### Fixing issues

If the CI fails check the pipeline logs, and fix packaging errors in the debian/master branch. When this branch is pushed again, then the CI will trigger a new build. If errors are related to Shmdata itself, then a new corrected version must be produced. 