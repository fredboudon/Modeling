import openalea.plantgl.all as pgl
from numpy import *

scenelist = []

black, blue, cyan, green, red, white, yellow = pgl.Color4.BLACK, pgl.Color4.BLUE, pgl.Color4.CYAN, pgl.Color4.GREEN, pgl.Color4.RED, pgl.Color4.WHITE, pgl.Color4.YELLOW


def update():
    pgl.Viewer.display(pgl.Scene())
    for scene in scenelist:
        pgl.Viewer.add(scene[1])
    pgl.Viewer.update()


def show(scene, color=black, width=1):
    if isinstance(scene, pgl.PointSet):
        scene.width = width
        scene.colorList = pgl.Color4Array(full(len(scene.pointList), color, dtype=pgl.Color4))
    scenelist.append((scene.getId(), scene))
    update()


def hide(scene):
    global scenelist
    scenelist = filter(lambda x: x[0] != scene.getId(), scenelist)
    update()


def clear():
    global scenelist
    scenelist = []
    update()


def move(pointset, position=0, zoom=1):
    ps = []
    for p in pointset.pointList:
        p = multiply(p, zoom) + position
        ps.append(p)
    return pgl.PointSet(ps)


def start():
    pgl.Viewer.start()


def stop():
    pgl.Viewer.stop()


def restart():
    pgl.Viewer.start()
    pgl.Viewer.grids.setAxis(False)
    clear()
    pgl.Viewer.stop()


restart()
