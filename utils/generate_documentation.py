#!/usr/bin/env python3

import os
import subprocess
import clean


if __name__ == "__main__":
    clean.removeDocumentation()
    os.chdir("..")
    subprocess.run("doxygen Doxyfile", shell=True)
