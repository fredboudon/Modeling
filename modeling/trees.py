from openalea.lpy import Lsystem
from modeling.gui import *
import random


def createprettytree():
    lsys = Lsystem("/home/fournierr/anaconda3/envs/moddeling/lib/python2.7/site-packages/OpenAlea.Lpy-2.7.1-py2.7-linux-x86_64.egg/share/tutorial/04 - simple-plant-archi/02 - random-tree.lpy")
    lstring = lsys.iterate()
    lscene = lsys.sceneInterpretation(lstring)
    return lscene


# return a scene of a basic tree
def createsaguaro(derivlength=15, trunc_rad=0.2, trunc_length=5, elasticity=0.05, A_1=80, branch_rad=0.1, rotation=137.5, B_1=0.8, B_2=0.1):
    lsys = Lsystem()
    lsys.axiom = "_(" + str(trunc_rad) + ") F(" + str(trunc_length) + ") @Ts(" + str(elasticity) + ")  A"
    lsys.derivationLength = derivlength
    lsys.addRule("A --> [ ^(" + str(A_1) + ") _(" + str(branch_rad) + ") B] /(" + str(rotation) + ") F A")
    lsys.addRule("B --> nF(" + str(B_1) + "," + str(B_2) + ") B")
    lstring = lsys.iterate()
    lscene = lsys.sceneInterpretation(lstring)
    return lscene
