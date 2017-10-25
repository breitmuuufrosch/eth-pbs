@echo off

set CMAKE_GENERATOR="Visual Studio 14 2015"

mkdir build
cd build

cmake -G%CMAKE_GENERATOR% ../

cd ..