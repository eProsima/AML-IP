# eProsima AML-IP docker Image

This image is installed with an AML-IP that is able to run a demo nodes.

---

## Build Docker image

Move to this directory and run the following command.

```sh
docker build -t amlip --no-cache -f Dockerfile .
```

---

## Using this Docker image

### Instructions on how to run an already image

```sh
# Run docker image
docker run --rm -it --net=host --ipc=host amlip
```

### Instructions on how to build it

```sh
# Build docker image (from workspace where Dockerfile is)
docker build --rm -t amlip -f Dockerfile .
# use --no-cache argument to restart build
```
