#!/usr/bin/env python3

import os
import subprocess
import clean

build_dir = "./build"
base_dir = "./mcsim"

coverage_file_full = "coverage_full.info"
coverage_file = "coverage.info"


def executeAllSteps():
    print("Generating coverage report...")
    captureCoverage()
    removeExclusions()
    generateCoverageSummary()
    generateCoverageReport()
    print("Generating coverage report done.")


def captureCoverage():
    cmd = [
        "lcov", "--capture",
        "--base-directory", base_dir,
        "--no-external",
        "--directory", build_dir,
        "--output-file", coverage_file_full
    ]
    subprocess.run(cmd)


def getExclusions():
    file_path = 'utils/lcov_exclude.txt'
    with open(file_path, 'r') as file:
        exclusions = file.readlines()
    exclusions = [line.strip() for line in exclusions]
    return exclusions


def removeExclusions():
    exclusions = getExclusions()
    cmd = ["lcov", "--remove", coverage_file_full]
    for exclusion in exclusions:
        cmd.append(exclusion)
    subprocess.run(cmd + ["--output-file", coverage_file])
    clean.removeFile(coverage_file_full)


def generateCoverageSummary():
    subprocess.run(
        "lcov --summary" + coverage_file + " > coverage_summary.txt",
        shell=True
    )


def generateCoverageReport():
    cmd = [
        "genhtml", coverage_file,
        "--legend",
        "--function-coverage",
        "--output-directory", "coverage-report"
    ]
    subprocess.run(cmd)


if __name__ == "__main__":
    clean.removeCoverageReport()
    os.chdir("..")
    executeAllSteps()
