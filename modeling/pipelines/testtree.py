# %gui qt4
# import modeling.gui

from modeling import *

tree = load_pointset("/home/fournierr/Documents/Stage CIRAD/data/Winter trees/X0036-1-1-WW.txt")

pred = load_pointset("/home/fournierr/Documents/Stage CIRAD/data/testtree.txt")

show(pgl.PointSet(tree))
show(pgl.PointSet(pred), width=5, color=green)