```bash
cd dockerfiles; cat ci* build* clean* | docker build -t shmdata:develop -f- ..
```

```bash
CONTAINER_NAME=shmdata
CONTAINER_VERSION=develop

```

```bash
TARGET="clean"
DOCKERFILE=""

while [ -f "${TARGET}.Dockerfile" ]; do
    TARGET=$(cat ${TARGET}.Dockerfile | grep FROM | sed -E 's/^FROM\s(\S*).*$/\1/')
    DOCKERFILE+=$(cat ${TARGET}.Dockerfile)
done

echo "${DOCKERFILE}"
```