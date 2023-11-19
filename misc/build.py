#!/usr/bin/env python3

################################################################################

import os
import platform
import subprocess
import sys

################################################################################

build_dir = "build"

################################################################################


def clean():
    subprocess.run(["python3", "clean.py"])


def createBuildDir():
    os.makedirs(build_dir)


def build(with_tests):
    os.chdir(build_dir)
    if with_tests:
        print("Building with tests...")
    else:
        print("Building...")
    os_name = platform.system()
    if os_name == "Linux":
        buildForLinux(with_tests)
    elif os_name == "Windows":
        buildForWindows()
    print("Building done.")


def buildForLinux(with_tests):
    cmake_cmd = [
        'cmake', '..',
        '-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_INSTALL_PREFIX=/usr/local'
    ]
    if with_tests:
        cmake_cmd.append('-DTESTING=On')
        cmake_cmd.append('-DCMAKE_CXX_FLAGS=-O0 -fno-elide-constructors -fno-default-inline -fprofile-arcs -ftest-coverage')
    result = subprocess.run(cmake_cmd)
    if result.returncode == 0:
        subprocess.run("make -j 4", shell=True)


def buildForWindows():
    cmake_cmd = [
        'cmake', '..',
        '-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_INSTALL_PREFIX=%LIBMCUTILS_DIR%'
    ]
    result = subprocess.run(cmake_cmd)
    if result.returncode == 0:
        subprocess.run("cmake --build . --config Release", shell=True)


################################################################################


if __name__ == "__main__":
    clean()
    os.chdir("..")
    createBuildDir()
    with_tests = False
    if "--with-tests" in sys.argv:
        with_tests = True
    build(with_tests)
