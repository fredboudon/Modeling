# %gui qt4
# import modeling.gui

from modeling import *

tree = load_pointset("/home/fournierr/Documents/Stage CIRAD/data/Winter trees/X0036-1-1-WW.txt")

pred = load_pointset("/home/fournierr/Documents/Stage CIRAD/data/testtree.txt")

bbx = getbbx(pgl.PointSet(tree))
pos = - ((bbx[0] + bbx[1]) / 2)

show(move(pgl.PointSet(tree), position=pos))
show(move(pgl.PointSet(pred), position=pos), width=5, color=green)