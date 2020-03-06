# before running:
# %gui qt4
# import modeling.gui


from modeling import *

path = "syntectic_tree_01.bgeom"

pointsets = topointset(path)


def display(a, b):
    show_all((a, b), colors=(black, green), widths=(1, 5))
