#!/usr/bin/env python3

################################################################################

import numpy as np
import ussa1976

################################################################################

height = np.concatenate(
    [
        np.arange(0.0, 1000.0, 100.0),
        np.arange(1000.0, 80000.0, 1000.0)
    ]
)

ds = ussa1976.core.compute(z=height, variables=["t", "p", "rho", "cs", "mu", "nu"])

results = np.vstack((height, ds['t'].values, ds['p'].values, ds['rho'].values, ds['cs'].values, ds['mu'].values, ds['nu'].values))
results = results.transpose()

for result in results:
    output = str(result[0])
    output = output + " " + str(result[1])
    output = output + " " + str(result[2])
    output = output + " " + str(result[3])
    output = output + " " + str(result[4])
    output = output + " " + str(result[5])
    output = output + " " + str(result[6])
    print(output)