# before running:
# %gui qt4
# import modeling.gui


from modeling import *

paths = [
    "/home/fournierr/Documents/Stage CIRAD/data/test trees/saguaro_predict_0.1.txt",
    "/home/fournierr/Documents/Stage CIRAD/data/test trees/randomtree_predict_0.05.txt",
    "/home/fournierr/Documents/Stage CIRAD/data/test trees/appletree_predict_0.003.txt",
    "/home/fournierr/Documents/Stage CIRAD/data/test trees/cleanappletree_predict_0.003.txt",
    "/home/fournierr/Documents/Stage CIRAD/data/bgeoms/saguaro/saguaro_scan.bgeom",
    "/home/fournierr/Documents/Stage CIRAD/data/bgeoms/randomtree/randomtree_scan.bgeom",
    "/home/fournierr/Documents/Stage CIRAD/data/bgeoms/appletree/appletree_scan.bgeom",
    "/home/fournierr/Documents/Stage CIRAD/data/bgeoms/cleanappletree/cleanappletree_scan.bgeom"
]

pointsets = topointsets(paths)

saguaro_pred, randomtree_pred, appletree_pred, cleanappletree_pred, saguaro_scan, randomtree_scan, appletree_scan, cleanappletree_scan = pointsets

print "saguaro"
sag_scan, sag_pred = assemble([saguaro_scan, saguaro_pred])
print "randomtree"
rt_scan, rt_pred = assemble([randomtree_scan, randomtree_pred])
print "appletree"
at_scan, at_pred = assemble([appletree_scan, appletree_pred])
print "cleanappletree"
cat_scan, cat_pred = assemble([cleanappletree_scan, cleanappletree_pred])


def display(a, b):
    show_all((a, b), colors=(black, green), widths=(1, 5))
