# before running:
# %gui qt4
# import modeling.gui


from modeling import *


nbsaguaro = 10

for i in range(nbsaguaro):

    clear()

    saguaro = createsaguaro()
    show(saguaro)

    #scan = surfacescan(saguaro, .0001)
    #scan = filledscan(saguaro, .00003)

    scan = lidarscan(saguaro, 90)
    skel = skeleton(saguaro, threshold=0.01)

    clear()

    show(scan)
    show(skel, width=5, color=green)

    #bbx = pgl.BoundingBox(saguaro)


    ### DATA TRANSFER ###

    #name = "_".join(map(str, [derivlength, trunc_rad, trunc_length, elasticity, A_1, branch_rad, rotation, B_1, B_2]))
    #directory = "/home/fournierr/Documents/Stage CIRAD/data/saguaros_2/"
    #savez(directory + name, scan=scan.pointList, skel=skel.pointList, bounds=(bbx.lowerLeftCorner, bbx.upperRightCorner))

#stop()