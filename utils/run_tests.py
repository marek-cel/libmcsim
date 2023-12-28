#!/usr/bin/env python3

import os
import platform
import subprocess
import misc


def runTestsOnLinux():
    setLibraryPathOnLinux()
    os.chdir("bin")
    subprocess.run("./tests --gtest_filter=* --gtest_break_on_failure", shell=True)


def runTestsOnWindows():
    setLibraryPathOnWindows()
    os.chdir("bin")
    subprocess.run("tests.exe --gtest_filter=* --gtest_break_on_failure", shell=True)


def setLibraryPathOnLinux():
    ld_library_path = str(os.environ.get("LD_LIBRARY_PATH"))
    ld_library_path = ld_library_path + ":" + str(os.path.join(os.getcwd(), "lib"))
    ld_library_path = ld_library_path + ":/usr/local/lib"
    os.environ["LD_LIBRARY_PATH"] = ld_library_path


def setLibraryPathOnWindows():
    path = str(os.environ.get("PATH"))
    gtest_dir = str(os.environ.get("GTEST_DIR"))
    path = path + ";" + str(os.path.join(gtest_dir, "bin"))
    os.environ["PATH"] = path


if __name__ == "__main__":
    misc.printGreen("Running tests...")
    os.chdir("..")
    os_name = platform.system()
    if os_name == "Linux":
        runTestsOnLinux()
    elif os_name == "Windows":
        runTestsOnWindows()
    misc.printGreen("Running tests done.")
