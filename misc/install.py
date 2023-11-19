#!/usr/bin/env python3

################################################################################

import os
import platform
import subprocess
import sys

################################################################################

build_dir = "build"

################################################################################


def build():
    subprocess.run(["python3", "build.py"])


def install():
    os.chdir(build_dir)
    print("Installing...")
    os_name = platform.system()
    if os_name == "Linux":
        installForLinux()
    elif os_name == "Windows":
        installForWindows()
    print("Installing done.")


def installForLinux():
    subprocess.run("sudo make install", shell=True)


def installForWindows():
    subprocess.run("cmake --build . --config Release --target INSTALL", shell=True)


################################################################################


if __name__ == "__main__":
    build()
    os.chdir("..")
    install()
