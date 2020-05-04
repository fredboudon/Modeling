# before running:
# %gui qt4
# ==============================================================================
from openalea.plantgl.all import *
from modeling import *
import os.path
import glob
import numpy
import pandas
import collections
# ==============================================================================


def get_geometry(sh):
    if hasattr(sh, 'geometry'):
        return get_geometry(sh.geometry)
    elif hasattr(sh, 'primitive'):
        return get_geometry(sh.primitive)
    else:
        return sh


def my_lidarscan(scene, a=90, z=1):
    # pgl.Viewer.display(scene)
    # sc = pgl.Viewer.getCurrentScene()
    bbx = pgl.BoundingBox(scene)
    c = bbx.getCenter()
    p, h, u = pgl.Viewer.camera.getPosition()
    pts = pgl.PointSet([], [])

    # pgl.Viewer.camera.setPosition(pgl.Vector3(100, 100, 8))
    # p, h, u = pgl.Viewer.camera.getPosition()
    # print(p, h, u)
    # pgl.Viewer.camera.lookAt(pgl.Vector3(100, 100, 8),
    #                          c)
    # pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
    # pts.pointList += pi
    # pts.colorList += ci

    for a in arange(0, 360, a):
        np = (c + pgl.Matrix3.axisRotation(pgl.Vector3(0, 0, 1), numpy.double(a))
              * pgl.Vector3(1, 0, 0)
              * pgl.norm(p-c))

        print(np)
        pgl.Viewer.camera.lookAt(np, c)
        jitter = 0.1
        raywidth = 2
        pi, ci = pgl.Viewer.frameGL.grabZBufferPoints(jitter, raywidth)
        # pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
        pts.pointList += pi
        pts.colorList += ci

    return pts


def convert_synthetic_to_point_cloud(filename, output_dir="synthetic_tree_labeled"):

    black = Material((0, 0, 0))
    red = Material((255, 0, 0))

    number_of_apple = 0
    cleaned_scene = Scene()
    for shape in Scene(filename):
        if shape.id != 0:
            cleaned_scene.add(Shape(shape.geometry,
                                    shape.appearance,
                                    shape.id,
                                    shape.parentId))

    Viewer.display(cleaned_scene)
    return None

    black_scene = Scene()
    for shape in cleaned_scene:
        if shape.id != 0:
            if isinstance(get_geometry(shape), Sphere):
                number_of_apple += 1
                black_scene.add(Shape(shape.geometry,
                                      red,
                                      shape.id,
                                      shape.parentId))
            else:
                black_scene.add(Shape(shape.geometry,
                                      black,
                                      shape.id,
                                      shape.parentId))

    Viewer.grids.set(False, False, False, False)

    pgl.Viewer.display(black_scene)

    scan = my_lidarscan(black_scene)

    pts = numpy.array(scan.pointList)
    colors = numpy.array([(c.red, c.green, c.blue) for c in scan.colorList])

    label = colors[:, 0] >= 200
    data = numpy.concatenate([pts, label.reshape(-1, 1)], axis=1)
    data[:, :3] /= 10

    number_apple_point = numpy.count_nonzero(label)

    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    basename = os.path.basename(os.path.splitext(filename)[0])
    output_filename = os.path.join(output_dir, "{}.txt".format(basename))
    numpy.savetxt(output_filename, data)

    return number_of_apple, number_apple_point


"""

ipython
%gui qt
from openalea.plantgl.all import *
Viewer.display(Scene())
%run syn...
"""


if __name__ == "__main__":

    input_dir = "/home/artzet_s/code/dataset/synthetic_data/synthetic_MAppleT/1996_9"
    output_dir = "/home/artzet_s/code/dataset/synthetic_data/synthetic_lidar_simulation_new"

    Viewer.grids.set(False, False, False, False)

    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    d = collections.defaultdict(list)
    for filename in glob.glob("{}/*.bgeom".format(input_dir))[:10]:

        data = convert_synthetic_to_point_cloud(filename,
                                                output_dir=output_dir)
        break

    # d['basename'].append(os.path.basename(os.path.splitext(filename)[0]))
    # d['number_of_apple'].append(data[0])
    # d['number_apple_point'].append(data[1])

    # df = pandas.DataFrame(d)
    # df.to_csv(os.path.join(output_dir, "synthetic_tree_apple_number.csv"))
