import openalea.plantgl.all as pgl
from numpy import *


# return a PointSet of a simulated LIDAR scan of the tree
# (ZBuffer points from different angles)
def lidarscan(scene, a=90, z=1):
    pgl.Viewer.display(scene)
    sc = pgl.Viewer.getCurrentScene()
    bbx = pgl.BoundingBox(sc)
    c = bbx.getCenter()
    p, h, u = pgl.Viewer.camera.getPosition()
    pts = pgl.PointSet([], [])
    for a in arange(0, 360, a):
        np = (c + pgl.Matrix3.axisRotation((0, 0, 1), a)
              * pgl.Vector3(1, 0, 0)
              * pgl.norm(p-c))

        pgl.Viewer.camera.lookAt(np / z, c)
        pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
        pts.pointList += pi
        pts.colorList += ci

    return pts


# return a PointSet of the "filled" tree (random points inside the tree)
def filledscan(scene, density):
    result = []
    for shape in scene:
        position = pgl.Vector3(0, 0, 0)
        heading = pgl.Vector3(0, 0, 1)
        geometry = shape.geometry
        while isinstance(geometry, pgl.Transformed):
            if isinstance(geometry, pgl.Translated):
                position = geometry.translation
            if isinstance(geometry, pgl.Oriented):
                heading = cross(geometry.primary, geometry.secondary)
            geometry = geometry.geometry
        if isinstance(geometry, pgl.Cylinder):
            for p in range(int((pi*(geometry.radius**2)*geometry.height) // density)):
                v = pgl.Vector3(random.rand() * 2 - 1,
                                random.rand() * 2 - 1, random.rand() * 2 - 1)
                pos = position + \
                    pgl.Vector3(cross(heading, v)).normed() * \
                    (geometry.radius*sqrt(random.rand()))
                point = pos+heading*random.rand()*geometry.height
                result.append(point)
        elif isinstance(geometry, pgl.Extrusion):
            result.append(geometry.axis)
    result = pgl.PointSet(result)
    return result


# return a PointSet of the "emptied" tree (random points on the surface of the tree)
def surfacescan(scene, density):
    result = []
    for shape in scene:
        position = pgl.Vector3(0, 0, 0)
        heading = pgl.Vector3(0, 0, 1)
        geometry = shape.geometry
        while isinstance(geometry, pgl.Transformed):
            if isinstance(geometry, pgl.Translated):
                position = geometry.translation
            if isinstance(geometry, pgl.Oriented):
                heading = cross(geometry.primary, geometry.secondary)
            geometry = geometry.geometry
        if isinstance(geometry, pgl.Cylinder):
            for p in range(int((pi*(geometry.radius**2)*geometry.height) // density)):
                v = pgl.Vector3(random.rand() * 2 - 1,
                                random.rand() * 2 - 1, random.rand() * 2 - 1)
                pos = position + \
                    pgl.Vector3(cross(heading, v)).normed() * (geometry.radius)
                point = pos+heading*random.rand()*geometry.height
                result.append(point)
        elif isinstance(geometry, pgl.Extrusion):
            result.append(geometry.axis)
    result = pgl.PointSet(result)
    return result


# return a PointSet of the skeleton of the tree
def skeleton(scene, threshold=0.01):
    result = []
    for shape in scene:
        position = pgl.Vector3(0, 0, 0)
        heading = pgl.Vector3(0, 0, 1)
        geometry = shape.geometry
        while isinstance(geometry, pgl.Transformed):
            if isinstance(geometry, pgl.Translated):
                position = geometry.translation
            if isinstance(geometry, pgl.Oriented):
                heading = cross(geometry.primary, geometry.secondary)
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


# check if a point is contained in a cylinder
def isincylinder(point, base, dir, radius, height):
    return 0 <= dot(point - base, pgl.Vector3(dir).normed()) <= height and pgl.norm(cross(point - base, pgl.Vector3(dir).normed())) < radius
