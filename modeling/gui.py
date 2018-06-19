import openalea.plantgl.all as pgl
from numpy import *

# list of the currently displayed pgl scenes
scenelist = []

# colors variables
black, blue, cyan, green, red, white, yellow = pgl.Color4.BLACK, pgl.Color4.BLUE, pgl.Color4.CYAN, pgl.Color4.GREEN, pgl.Color4.RED, pgl.Color4.WHITE, pgl.Color4.YELLOW


# display a pointset in the viewer
def show(scene, color=black, width=1, clearprevious=False):
    if isinstance(scene, pgl.PointSet):
        scene.width = width
        scene.colorList = pgl.Color4Array(full(len(scene.pointList), color, dtype=pgl.Color4))
    scenelist.append((scene.getId(), scene))
    if clearprevious: clear()
    update()


# remove a displayed scene from the viewer
def hide(scene):
    global scenelist
    scenelist = filter(lambda x: x[0] != scene.getId(), scenelist)
    update()


# remove all displayed scenes from the viewer
def clear():
    global scenelist
    scenelist = []
    update()


# update the displayed scenes so that only the scenes of scenelist are displayed
def update():
    pgl.Viewer.display(pgl.Scene())
    for scene in scenelist:
        pgl.Viewer.add(scene[1])
    pgl.Viewer.update()


# restart the viewer
def restart():
    pgl.Viewer.start()
    pgl.Viewer.grids.setAxis(False)
    clear()
    pgl.Viewer.stop()


# one restart needed before using the gui
restart()
