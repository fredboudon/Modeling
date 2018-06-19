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


def createprettytree():
    return createtree(os.path.dirname(inspect.getfile(modeling)) + "/lpy_trees/pretty_tree.lpy")


def createsaguaro():
    return createtree(os.path.dirname(inspect.getfile(modeling)) + "/lpy_trees/saguaro.lpy")


# return a scene of a basic tree
def createsaguaro2(derivlength=15, trunc_rad=0.2, trunc_length=5, elasticity=0.05, A_1=80, branch_rad=0.1, rotation=137.5, B_1=0.8, B_2=0.1):
    lsys = Lsystem()
    lsys.axiom = "_(" + str(trunc_rad) + ") F(" + str(trunc_length) + ") @Ts(" + str(elasticity) + ")  A"
    lsys.derivationLength = derivlength
    lsys.addRule("A --> [ ^(" + str(A_1) + ") _(" + str(branch_rad) + ") B] /(" + str(rotation) + ") F A")
    lsys.addRule("B --> nF(" + str(B_1) + "," + str(B_2) + ") B")
    lstring = lsys.iterate()
    lscene = lsys.sceneInterpretation(lstring)
    return lscene
