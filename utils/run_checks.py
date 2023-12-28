#!/usr/bin/env python3

import os
import subprocess
import clean
import misc

build_dir = "build"
src_dir = "mcsim"


def run_cloc():
    subprocess.run("cloc ./" + src_dir + "/* > out_cloc.txt", shell=True)


def run_cppcheck():
    command = "cppcheck"
    options = "--enable=style --std=c++17 -I ./" + src_dir + " ./" + src_dir
    subprocess.run(
        command + " " + options + " 2> out_cppcheck.txt",
        shell=True
    )


def run_cpplint():
    filter = "-build/c++11,+build/c++14,+build/c++17" \
        + ",-readability/multiline_comment" \
        + ",-readability/alt_tokens" \
        + ",-whitespace,-legal/copyright" \
        + ",-build/include_order"
    cmd = "cpplint" \
        + " --filter=" + filter \
        + " --linelength=100" \
        + " --extensions=h,cpp" \
        + " --root=./" \
        + " ./" + src_dir + "/*.h" \
        + " ./" + src_dir + "/*/*.h" \
        + " ./" + src_dir + "/*/*.cpp" \
        + " 2> out_cpplint.txt"
    subprocess.run(cmd, shell=True)


def run_pycodestyle():
    for file in os.listdir("."):
        if file.endswith('.py'):
            command = "pycodestyle"
            options = "--show-source --show-pep8 --max-line-length=100"
            subprocess.run(
                command + " " + options + " " + file + " >> ../out_pep.txt",
                shell=True
            )


def runScanBuild():
    clean.removeBuildDirs()
    os.makedirs(build_dir)
    os.chdir(build_dir)
    cmd = [
        "cmake", "..",
        "-DCMAKE_BUILD_TYPE=Release"
    ]
    subprocess.run(cmd)
    subprocess.run("scan-build-10 -analyze-headers make -j 4", shell=True)


################################################################################

if __name__ == "__main__":
    clean.removeCheckOutputs()
    misc.printGreen("Running checks...")
    run_pycodestyle()
    os.chdir("..")
    run_cloc()
    run_cppcheck()
    run_cpplint()
    misc.printGreen("Running checks done.")
