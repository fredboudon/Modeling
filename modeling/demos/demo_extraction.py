# before running:
# %gui qt4
# import modeling.gui

from modeling import *

tree = createsaguaro()

show(tree)

scan = lidarscan(tree)

skel = skeleton(tree)

clear()

show(scan)

show(skel, width=3, color=green)

bbx = getbbx(tree)

savez("mytree", scan=scan.pointList, skel=skel.pointList, bounds=bbx)
