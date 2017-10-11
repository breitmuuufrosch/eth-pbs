#!/bin/bash

mkdir -p build
cd build

mkdir -p Debug
cd Debug
cmake -G"Unix Makefiles" -D CMAKE_BUILD_TYPE="Debug" ../..

cd ..
mkdir -p Release
cd Release
cmake -G"Unix Makefiles" -D CMAKE_BUILD_TYPE="Release" ../..
