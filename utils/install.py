#!/usr/bin/env python3

import os
import platform
import subprocess
import misc

build_dir = "build"


def build():
    subprocess.run(["python3", "build.py"])


def install():
    misc.printGreen("Installing...")
    os_name = platform.system()
    if os_name == "Linux":
        installForLinux()
    elif os_name == "Windows":
        installForWindows()
    misc.printGreen("Installing done.")


def installForLinux():
    subprocess.run(
        "sudo cmake --build " + build_dir + " --config Release --target install",
        shell=True
    )


def installForWindows():
    subprocess.run(
        "cmake --build " + build_dir + " --config Release --target INSTALL",
        shell=True
    )


if __name__ == "__main__":
    build()
    os.chdir("..")
    install()
