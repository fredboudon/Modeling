from openalea.lpy import Lsystem
from modeling.gui import *

import inspect
import os
import modeling


# run the lpy file found at path and return the scene of the resulting tree
def createtree(path):
    lsys = Lsystem(path)
    lstring = lsys.iterate()
    lscene = lsys.sceneInterpretation(lstring)
    return lscene


# return the scene of a random "pretty" tree
def createprettytree():
    return createtree(os.path.dirname(inspect.getfile(modeling)) + "/lpy_trees/pretty_tree.lpy")


# return the scene of a random saguaro
def createsaguaro():
    return createtree(os.path.dirname(inspect.getfile(modeling)) + "/lpy_trees/saguaro.lpy")

