from numpy import *
import openalea.plantgl.all as pgl


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


def move(pointset, position=0, zoom=1):
    ps = []
    for p in pointset.pointList:
        p = multiply(p, zoom) + position
        ps.append(p)
    return pgl.PointSet(ps)