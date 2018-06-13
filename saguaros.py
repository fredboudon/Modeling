# before running:
# %gui qt4
# import gui

import openalea.plantgl.all as pgl
from gui import *
from tree import *

nbsaguaro = 1

for i in range(nbsaguaro):

    clear()

    ### CONSTRUCTION ###

    # saguaro parameters

    derivlength = random.randint(8, 12)
    trunc_rad = float(random.randint(20, 30)) / 100
    trunc_length = random.randint(3, 5)
    elasticity = float(random.randint(10, 50)) / 1000
    A_1 = 80
    branch_rad = float(random.randint(10, 20)) / 100
    rotation = random.randint(100, 150)
    B_1 = 0.8
    B_2 = 0.1

    # flat saguaro parameters

    # derivlength = random.randint(8, 12)
    # trunc_rad = float(random.randint(20, 30)) / 100
    # trunc_length = random.randint(3, 5)
    # elasticity = float(random.randint(10, 50)) / 1000
    # A_1 = 80
    # branch_rad = float(random.randint(10, 20)) / 100
    # rotation = 180
    # B_1 = 0.8
    # B_2 = 0.1


    #saguaro = createsaguaro(derivlength=derivlength, trunc_rad=trunc_rad, trunc_length=trunc_length, elasticity=elasticity, A_1=A_1, branch_rad=branch_rad, rotation=rotation, B_1=B_1, B_2=B_2)[1]

    saguaro = createsaguaro(derivlength=10, trunc_rad=0.25, trunc_length=3, elasticity=0.05, A_1=80, branch_rad=0.15, rotation=137.5, B_1=0.8, B_2=0.1)[1]


    show(saguaro)

    #scan = surfacescan(saguaro, .0001)
    #scan = filledscan(saguaro, .00003)

    scan = lidarscan(saguaro, 90)
    skel = skeleton(saguaro, threshold=0.01)

    clear()

    show(scan)
    show(skel, width=5, color=green)

    #bbx = pgl.BoundingBox(saguaro)



    ## DATA TRANSFER ###

    #name = "_".join(map(str, [derivlength, trunc_rad, trunc_length, elasticity, A_1, branch_rad, rotation, B_1, B_2]))
    #directory = "/home/fournierr/Documents/Stage CIRAD/data/saguaros_2/"
    #savez(directory + name, scan=scan.pointList, skel=skel.pointList, bounds=(bbx.lowerLeftCorner, bbx.upperRightCorner))

#stop()