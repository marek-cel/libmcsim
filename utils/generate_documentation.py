#!/usr/bin/env python3

import os
import subprocess
import clean
import misc


if __name__ == "__main__":
    clean.removeDocumentation()
    os.chdir("..")
    misc.printGreen("Generating documentation...")
    subprocess.run("doxygen Doxyfile", shell=True)
    misc.printGreen("Generating documentation done.")
