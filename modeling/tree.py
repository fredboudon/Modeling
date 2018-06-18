from openalea.lpy import Lsystem
from modeling.gui import *
import random


def createprettytree():
    lsys = Lsystem("/home/fournierr/anaconda3/envs/moddeling/lib/python2.7/site-packages/OpenAlea.Lpy-2.7.1-py2.7-linux-x86_64.egg/share/tutorial/04 - simple-plant-archi/02 - random-tree.lpy")
    lstring = lsys.iterate()
    lscene = lsys.sceneInterpretation(lstring)
    # return lstring, lscene
    return lscene


# return a lstring and a scene of a basic tree
def createsaguaro(derivlength=15, trunc_rad=0.2, trunc_length=5, elasticity=0.05, A_1=80, branch_rad=0.1, rotation=137.5, B_1=0.8, B_2=0.1):
    lsys = Lsystem()
    lsys.axiom = "_(" + str(trunc_rad) + ") F(" + str(trunc_length) + ") @Ts(" + str(elasticity) + ")  A"
    lsys.derivationLength = derivlength
    lsys.addRule("A --> [ ^(" + str(A_1) + ") _(" + str(branch_rad) + ") B] /(" + str(rotation) + ") F A")
    lsys.addRule("B --> nF(" + str(B_1) + "," + str(B_2) + ") B")
    lstring = lsys.iterate()
    lscene = lsys.sceneInterpretation(lstring)
    # return lstring, lscene
    return lscene


# return a PointSet of a simulated scan of the tree
def lidarscan(scene, a=90, z=1):
    pgl.Viewer.display(scene)
    sc = pgl.Viewer.getCurrentScene()
    bbx = pgl.BoundingBox(sc)
    c = bbx.getCenter()
    p,h,u = pgl.Viewer.camera.getPosition()
    pts = pgl.PointSet([], [])
    for a in arange(0,360,a):
        np = c + pgl.Matrix3.axisRotation((0,0,1),a)*pgl.Vector3(1,0,0)*pgl.norm(p-c)
        pgl.Viewer.camera.lookAt(np/z,c)
        pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
        pts.pointList += pi
        pts.colorList += ci
    return pts


# return a PointSet of a filled tree
def filledscan(scene, density):
    result = []
    for shape in scene:
        position = pgl.Vector3(0,0,0)
        heading = pgl.Vector3(0,0,1)
        geometry = shape.geometry
        while isinstance(geometry, pgl.Transformed):
            if isinstance(geometry, pgl.Translated):
                position = geometry.translation
            if isinstance(geometry, pgl.Oriented):
                heading = cross(geometry.primary,geometry.secondary)
            geometry = geometry.geometry
        if isinstance(geometry, pgl.Cylinder):
            for p in range(int((pi*(geometry.radius**2)*geometry.height) // density)):
                v = pgl.Vector3(random.rand() * 2 - 1, random.rand() * 2 - 1, random.rand() * 2 - 1)
                pos = position + pgl.Vector3(cross(heading, v)).normed() * (geometry.radius*sqrt(random.rand()))
                point = pos+heading*random.rand()*geometry.height
                result.append(point)
        elif isinstance(geometry, pgl.Extrusion):
            result.append(geometry.axis)
    result = pgl.PointSet(result)
    return result


# return a PointSet of a filled tree
def surfacescan(scene, density):
    result = []
    for shape in scene:
        position = pgl.Vector3(0,0,0)
        heading = pgl.Vector3(0,0,1)
        geometry = shape.geometry
        while isinstance(geometry, pgl.Transformed):
            if isinstance(geometry, pgl.Translated):
                position = geometry.translation
            if isinstance(geometry, pgl.Oriented):
                heading = cross(geometry.primary,geometry.secondary)
            geometry = geometry.geometry
        if isinstance(geometry, pgl.Cylinder):
            for p in range(int((pi*(geometry.radius**2)*geometry.height) // density)):
                v = pgl.Vector3(random.rand() * 2 - 1, random.rand() * 2 - 1, random.rand() * 2 - 1)
                pos = position + pgl.Vector3(cross(heading, v)).normed() * (geometry.radius)
                point = pos+heading*random.rand()*geometry.height
                result.append(point)
        elif isinstance(geometry, pgl.Extrusion):
            result.append(geometry.axis)
    result = pgl.PointSet(result)
    return result


def isincylinder(p, base, dir, radius, height):
    return 0 <= dot(p - base, pgl.Vector3(dir).normed()) <= height and pgl.norm(cross(p - base, pgl.Vector3(dir).normed())) < radius


# return a PointSet of the skeleton of the tree
def skeleton(scene, threshold=0.01):
    result = []
    for shape in scene:
        position = pgl.Vector3(0,0,0)
        heading = pgl.Vector3(0,0,1)
        geometry = shape.geometry
        while isinstance(geometry, pgl.Transformed):
            if isinstance(geometry, pgl.Translated):
                position = geometry.translation
            if isinstance(geometry, pgl.Oriented):
                heading = cross(geometry.primary,geometry.secondary)
            geometry = geometry.geometry
        if isinstance(geometry, pgl.Cylinder) or isinstance(geometry, pgl.Cone):
            line = pgl.Polyline([position, position+heading*geometry.height])
            splits = max(1, line.getLength() / threshold)
            for i in linspace(0, 1, splits):
                subline = line.split(i)[0]
                result.append(subline.pointList[1])
        elif isinstance(geometry, pgl.Extrusion):
            result.append(geometry.axis)
    result = pgl.PointSet(result)
    return result


def load_pointset(path):
    return array(loadtxt(path))


def getbbx(scene):
    bbx = pgl.BoundingBox(scene)
    return bbx.lowerLeftCorner, bbx.upperRightCorner

def assemble(pathtree, pathresult):

    tree = load_pointset(pathtree)

    pred = load_pointset(pathresult)

    bbx = getbbx(pgl.PointSet(tree))
    pos = - ((bbx[0] + bbx[1]) / 2)

    res_tree = move(pgl.PointSet(tree), position=pos)
    res_result = move(pgl.PointSet(pred), position=pos)

    return res_tree, res_result