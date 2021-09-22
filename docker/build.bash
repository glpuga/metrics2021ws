#!/usr/bin/env bash

#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

IMAGE_NAME="metrics2021"
DOCKERFILE_PATH="."

if [ ! -f $DOCKERFILE_PATH/Dockerfile ]; then
    echo "Can't find the docker file in " $DOCKERFILE_PATH
    exit 1
fi

TAGGED_IMAGE_NAME=$IMAGE_NAME:$(
    export LC_ALL=C
    date +%Y_%m_%d_%H%M
)

# Build devel environment
docker build \
    --target devel_environment \
    --rm \
    -t $TAGGED_IMAGE_NAME \
    -f "$DOCKERFILE_PATH"/Dockerfile \
    "$DOCKERFILE_PATH" &&
    docker tag \
        $TAGGED_IMAGE_NAME \
        $IMAGE_NAME:latest &&
    echo "Built $TAGGED_IMAGE_NAME and tagged as $IMAGE_NAME:latest"

if [[ $? -ne 0 ]]; then
    echo "Error building image, aborting..."
    exit
fi

#
# Build releases
#

# first-submission
SUBMISSION="first-submission"
echo Building "$SUBMISSION..." &&
    docker build \
        --target $SUBMISSION \
        --build-arg NOCACHE_SUBMISSIONS=$(date +%s) \
        --rm \
        -t $TAGGED_IMAGE_NAME \
        -f "$DOCKERFILE_PATH"/Dockerfile \
        "$DOCKERFILE_PATH" &&
    docker tag \
        $TAGGED_IMAGE_NAME \
        $IMAGE_NAME:$SUBMISSION &&
    echo "Built $TAGGED_IMAGE_NAME and tagged as $IMAGE_NAME:$SUBMISSION"

# third-submission
SUBMISSION="third-submission"
echo Building "$SUBMISSION..." &&
    docker build \
        --target $SUBMISSION \
        --build-arg NOCACHE_SUBMISSIONS=$(date +%s) \
        --rm \
        -t $TAGGED_IMAGE_NAME \
        -f "$DOCKERFILE_PATH"/Dockerfile \
        "$DOCKERFILE_PATH" &&
    docker tag \
        $TAGGED_IMAGE_NAME \
        $IMAGE_NAME:$SUBMISSION &&
    echo "Built $TAGGED_IMAGE_NAME and tagged as $IMAGE_NAME:$SUBMISSION"

# fourth-submission
SUBMISSION="fourth-submission"
echo Building "$SUBMISSION..." &&
    docker build \
        --target $SUBMISSION \
        --build-arg NOCACHE_SUBMISSIONS=$(date +%s) \
        --rm \
        -t $TAGGED_IMAGE_NAME \
        -f "$DOCKERFILE_PATH"/Dockerfile \
        "$DOCKERFILE_PATH" &&
    docker tag \
        $TAGGED_IMAGE_NAME \
        $IMAGE_NAME:$SUBMISSION &&
    echo "Built $TAGGED_IMAGE_NAME and tagged as $IMAGE_NAME:$SUBMISSION"
