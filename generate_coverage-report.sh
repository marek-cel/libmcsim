#!/bin/bash

################################################################################

BUILD_DIR=./build
BASE_DIR=./mcsim

if [ ! -z "$1" ]
  then
    BUILD_DIR="${1%/}"
fi

################################################################################

rm -f coverage_full.info
rm -f coverage.info
rm -r -f coverage-report
mapfile -t exclude < lcov_exclude.txt
lcov --capture  \
    --base-directory $BASE_DIR \
    --no-external \
    --directory $BUILD_DIR \
    --output-file coverage_full.info
lcov --remove coverage_full.info \
    ${exclude[@]} \
    --output-file coverage.info
genhtml coverage.info \
    --legend \
    --function-coverage \
    --output-directory coverage-report

################################################################################
