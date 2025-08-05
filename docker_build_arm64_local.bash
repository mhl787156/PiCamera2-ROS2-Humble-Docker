#!/bin/bash

usage() {
    echo "  options:"
    echo "      -i: initialise"
    echo "      -n: docker name, defaults to osprey_imu_driver_ros2"
    echo "      -r: registry, defaults to 128.16.29.85:5000"
}

# Initialize variables with default values
init="false"
name="pi-camera-ros-driver"
registry="128.16.29.85:5000"

# Arg parser
while getopts "in:r:" opt; do
  case ${opt} in
    i )
      init="true"
      ;;
    n )
      name="${OPTARG}"
      ;;
    r )
      registry="${OPTARG}"
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

if [[ ${init} == "true" ]]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
    docker buildx create \
        --name container-builder \
        --driver docker-container \
        --config config/buildkitd.toml \
        --bootstrap --use
fi

FULL_NAME="${registry}/${name}:latest"

docker buildx build --builder=container-builder \
                    --platform linux/amd64,linux/arm64/v8 \
                    -t "${FULL_NAME}" --push .
