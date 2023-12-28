#!/usr/bin/env python3

import os
import shutil


def printGreen(text):
    print('\033[92m' + text + '\033[0m')


def removeDir(dir_path):
    if os.path.exists(dir_path):
        shutil.rmtree(dir_path)


def removeFile(file_path):
    if os.path.exists(file_path):
        os.remove(file_path)
