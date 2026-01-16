#!/bin/bash
rm -rf build
mkdir build
cd build
cmake -DUSE_MOTOR_TEST=ON ..
make -j
