#!/usr/bin/env python
# CREATE LOG BEFORE EXC
# run: ./exercise1_linux -testcase error_measurement -step 0.5 -damp 0 > log.txt

import numpy as np
import matplotlib.pyplot as plt
# run
# read the file
file = open("log.txt", "r") 
content = file.readlines()
velocity =  content[2:12]
displacement =  content[14:24]

# step euler symplectic_euler midpoint backwards_euler analytic
for i in range(10):
    velocity[i] = map(float, velocity[i].strip().split(" "))
    displacement[i] = map(float, displacement[i].strip().split(" "))

# create matrix
v = np.array(velocity)
d = np.array(displacement)

# velocity
# create materix where the colums are the analytic solution 
def getErrorMatrix(m):
    ana =  np.matrix(m[:, 5]).T
    # concat them to a bit matrix 
    analyticMV = np.concatenate((ana, ana, ana, ana), axis=1)

    # remove the col with the step size
    dataMV = m[:, 1:5]

    # compute the error per step to the analytic solution
    return abs(dataMV - analyticMV)

errPerStepV = getErrorMatrix(v)
# displacement
errPerStepD = getErrorMatrix(d)

# compute the relative errors
relativeErrV = np.zeros((9, 4))
for m in range(4):
    for row in range(9):
        relativeErrV[row, m] = errPerStepV[row, m] / errPerStepV[row + 1, m]

# compute the relative errors
relativeErrD = np.zeros((9, 4))
for m in range(4):
    for row in range(9):
        relativeErrD[row, m] = errPerStepD[row, m] / errPerStepD[row + 1, m]




# compute avg
print "Average relative errors:"
print "euler symplectic_euler midpoint backwards_euler"
print np.average(relativeErrV, axis=0)
print np.average(relativeErrD, axis=0)

# plot the per step error
plt.plot(d[:,0], errPerStepD[:,0], label='euler')
plt.plot(d[:,0], errPerStepD[:,1], label='symp')
plt.plot(d[:,0], errPerStepD[:,2], label='mid')
plt.plot(d[:,0], errPerStepD[:,3], label='back')
plt.legend()
plt.show()