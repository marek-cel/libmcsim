#!/bin/bash

################################################################################

build_dir="${1%/}"

################################################################################

if [ ! -z "$build_dir" ]
then
    rm -f coverage_full.info
    rm -f coverage.info
    rm -r -f coverage-report
    cd bin; ./libmcutil_tests; cd ..;
    lcov --capture --base-directory ./mcutil --no-external --directory $build_dir --output-file coverage_full.info
    lcov --remove coverage_full.info \
        "$(pwd)/tests/*" \
        "*math/IIntegrator.h" \
        "*signal/ISignalElement.h" \
        -o coverage.info
    genhtml coverage.info --output-directory coverage-report
else
    echo
    echo Error! Build directory path not specified.
    echo Usage:
    echo ./generate_coverage-report.sh BUILD_DIRECTORY_PATH
    echo
    read -p "Press any key to continue... " -n1 -s
    echo
fi

################################################################################
