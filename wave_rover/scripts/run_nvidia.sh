#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config.sh

rocker --x11 \
    --nvidia \
    --name=${CONTAINER_NAME} \
    --volume="${PROJECT_ROOT}:/${WS_ROS}/src" \
    -- ${DOCKER_IMAGE_NAME}
