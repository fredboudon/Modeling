from numpy import *
import openalea.plantgl.all as pgl


# convert an object or a path to txt or bgeom scene to a pointset
def topointset(obj):
    pointset = pgl.Scene(obj)[0] if isinstance(obj, str) else pgl.PointSet(obj)
    while not isinstance(pointset, pgl.PointSet):
        pointset = pointset.geometry
    return pointset


# convert multiple objects to pointsets
def topointsets(objs):
    return [topointset(obj) for obj in objs]


# save a scene to the indicated path (.txt, .xyz, .bgeom)
def save(scene, path):
    scene.save(path)


# return the boundingbox of a scene
def getbbx(scene):
    bbx = pgl.BoundingBox(scene)
    return bbx.lowerLeftCorner, bbx.upperRightCorner


# return the dimensions of a scene
def getdims(scene):
    bbx = getbbx(scene)
    return bbx[1] - bbx[0]


# move a pointset to a given position
def move(pointset, position):
    return pgl.PointSet(array(pointset.pointList) - getbbx(pointset)[0] + position)


# zoom a pointset to a given position
def zoom(pointset, zoom):
    position = getbbx(pointset)[0]
    return pgl.PointSet((array(pointset.pointList) - position) * zoom + position)


# zoom a pointset so it occupies the maximum possible space in the indicated bbx
def scale(pointset, bbx):
    dims, psdims = bbx[1] - bbx[0], getdims(pointset)
    return zoom(pointset, min(divide(dims, psdims)))


# deform a pointset to perfectly fit in the indicated bbx
def fit(pointset, bbx):
    dims, psdims = bbx[1] - bbx[0], getdims(pointset)
    return zoom(pointset, divide(dims, psdims))


# center a pointset
def center(pointset):
    return move(pointset, position=-getbbx(pointset)[0])


# center and scale the pointsets to the bbx
def assemble(pointsets):
    position = getbbx(pointsets[0])[0]
    return [move(pointset, position) for pointset in pointsets]