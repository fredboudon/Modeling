# before running:
# %gui qt4
# import gui

from modeling import *

nbtree = 1

for i in range(nbtree):

    clear()

    ### CONSTRUCTION ###

    tree = createprettytree()

    show(tree)

    # scan = surfacescan(tree, .0001)
    # scan = filledscan(tree, .00003)

    scan = lidarscan(tree, a=45, z=2 if i==0 else 1)
    skel = skeleton(tree, threshold=0.01)

    clear()

    show(scan)
    show(skel, width=3, color=green)

    #bbx = pgl.BoundingBox(tree)



    ## DATA TRANSFER ###

    #savez("/home/fournierr/Documents/Stage CIRAD/data/pretty trees/tree_{}".format(i), scan=scan.pointList, skel=skel.pointList, bounds=(bbx.lowerLeftCorner, bbx.upperRightCorner))

#stop()