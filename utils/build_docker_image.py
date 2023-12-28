#!/usr/bin/env python3

import subprocess
import misc


if __name__ == "__main__":
    misc.printGreen("Building Docker image...")
    dockerfile = '../Dockerfile'
    docker_cmd = [
        'docker', 'build',
        '-f',  dockerfile,
        '-t', 'libmcsim-dev-env',
        '.'
    ]
    print(" ".join(docker_cmd))
    subprocess.call(docker_cmd)
