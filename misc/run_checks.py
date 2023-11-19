#!/usr/bin/env python3

################################################################################

import os
import subprocess
import clean

################################################################################

build_dir = "build"
src_dir = "mcsim"

################################################################################


def run_cloc():
    subprocess.run("cloc ./" + src_dir + "/* > out_cloc.txt", shell=True)


def run_cppcheck():
    subprocess.run("cppcheck --enable=style --std=c++17 -I ./" + src_dir + " ./" + src_dir + " 2> out_cppcheck.txt", shell=True)


def run_cpplint():
    filter = "-build/c++11,+build/c++14,-readability/multiline_comment,-readability/alt_tokens,-whitespace/parens,-whitespace/braces,-whitespace/indent,-whitespace/comments,-whitespace/newline,-whitespace/blank_line,-whitespace/comma,-whitespace/line_length,-build/include_order"
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
    print("Running checks...")
    os.chdir("..")
    run_cloc()
    run_cppcheck()
    run_cpplint()
    print("Running checks done.")
