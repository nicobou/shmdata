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

