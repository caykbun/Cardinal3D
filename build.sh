#!/bin/bash

# Default build type
BUILD_TYPE="RelWithDebInfo"

# Check for --debug parameter
if [[ "$1" == "--debug" ]]; then
    BUILD_TYPE="DEBUG"
fi

# Commands to run
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j4
cd ..
