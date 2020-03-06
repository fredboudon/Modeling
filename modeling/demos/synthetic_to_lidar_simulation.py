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

    # pgl.Viewer.display(black_scene)

    scan = lidarscan(black_scene)

    pts = numpy.array(scan.pointList)
    colors = numpy.array([(c.red, c.green, c.blue) for c in scan.colorList])

    label = colors[:, 0] >= 200
    data = numpy.concatenate([pts, label.reshape(-1, 1)], axis=1)
    data[:, :3] /= 10

    number_apple_point = numpy.count_nonzero(label)

    if not os.path.exists(output_dir): os.mkdir(output_dir)
    basename = os.path.basename(os.path.splitext(filename)[0])
    output_filename = os.path.join(output_dir, "{}.txt".format(basename))
    numpy.savetxt(output_filename, data)

    return number_of_apple, number_apple_point


if __name__ == "__main__":

    input_dir = "/home/artzet_s/code/dataset/synthetic_MAppleT/1996_9"
    output_dir = "/home/artzet_s/code/dataset/synthetic_lidar_simulation"

    Viewer.grids.set(False, False, False, False)

    if not os.path.exists(output_dir): os.mkdir(output_dir)

    d = collections.defaultdict(list)
    for filename in glob.glob("{}/*.bgeom".format(input_dir)):

        data = convert_synthetic_to_point_cloud(filename,
                                                output_dir=output_dir)

        d['basename'].append(os.path.basename(os.path.splitext(filename)[0]))
        d['number_of_apple'].append(data[0])
        d['number_apple_point'].append(data[1])

    df = pandas.DataFrame(d)
    df.to_csv(os.path.join(output_dir, "synthetic_tree_apple_number.csv"))
